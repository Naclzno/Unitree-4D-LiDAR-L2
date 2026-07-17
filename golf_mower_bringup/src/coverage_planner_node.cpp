#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fields2cover.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

namespace golf_mower_bringup
{

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

enum class SegmentState
{
  PENDING,
  ACTIVE,
  COMPLETED,
  BLOCKED,
  CANCELED
};

struct CoverageSegment
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  SegmentState state{SegmentState::PENDING};
  int attempts{0};
};

struct NavWaypoint
{
  geometry_msgs::msg::PoseStamped pose;
  f2c::types::PathSectionType type{f2c::types::PathSectionType::SWATH};
};

struct AreaConfig
{
  std::string frame_id{"map"};
  std::vector<F2CPoint> boundary;
  std::vector<std::vector<F2CPoint>> exclusions;
  double robot_width{0.80};
  double coverage_width{0.70};
  double min_turning_radius{1.0};
  double cruise_speed{0.40};
  double turn_speed{0.20};
  int headland_swaths{3};
  bool use_fixed_swath_angle{false};
  double swath_angle_rad{0.0};
};

double signed_area(const std::vector<F2CPoint> & points)
{
  double area = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % points.size()];
    area += a.getX() * b.getY() - b.getX() * a.getY();
  }
  return 0.5 * area;
}

bool point_in_polygon(double x, double y, const std::vector<F2CPoint> & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double xi = polygon[i].getX();
    const double yi = polygon[i].getY();
    const double xj = polygon[j].getX();
    const double yj = polygon[j].getY();
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<F2CPoint> read_ring(const YAML::Node & node, const std::string & name)
{
  if (!node || !node.IsSequence() || node.size() < 3) {
    throw std::runtime_error(name + " must contain at least three [x, y] points");
  }

  std::vector<F2CPoint> points;
  points.reserve(node.size());
  for (size_t i = 0; i < node.size(); ++i) {
    const auto point = node[i];
    if (!point.IsSequence() || point.size() < 2) {
      throw std::runtime_error(name + " point " + std::to_string(i) + " must be [x, y]");
    }
    points.emplace_back(point[0].as<double>(), point[1].as<double>());
  }

  if (points.size() > 3 && points.front() == points.back()) {
    points.pop_back();
  }
  return points;
}

void orient_ring(std::vector<F2CPoint> & points, bool counter_clockwise)
{
  const bool is_counter_clockwise = signed_area(points) > 0.0;
  if (is_counter_clockwise != counter_clockwise) {
    // Fields2Cover 2.1 Point move-assignment is unsafe during std::reverse.
    std::vector<F2CPoint> reversed;
    reversed.reserve(points.size());
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
      reversed.emplace_back(it->getX(), it->getY(), it->getZ());
    }
    points.swap(reversed);
  }
}

std::string expand_user_path(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    throw std::runtime_error("HOME is not set; cannot expand output path " + path);
  }
  if (path.size() == 1) {
    return home;
  }
  if (path[1] != '/') {
    throw std::runtime_error("only ~/... output paths are supported");
  }
  return std::string(home) + path.substr(1);
}

class CoveragePlannerNode : public rclcpp::Node
{
public:
  CoveragePlannerNode()
  : Node("coverage_planner")
  {
    area_file_ = declare_parameter<std::string>("area_file", "");
    output_file_ = declare_parameter<std::string>(
      "output_file", "~/.ros/golf_mower/coverage_path.yaml");
    mission_state_file_ = declare_parameter<std::string>(
      "mission_state_file", "~/.ros/golf_mower/coverage_mission_state.yaml");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    auto_plan_ = declare_parameter<bool>("auto_plan", true);
    path_pose_spacing_ = declare_parameter<double>("path_pose_spacing", 0.10);
    nav_waypoint_spacing_ = declare_parameter<double>("nav_waypoint_spacing", 0.75);
    republish_period_sec_ = declare_parameter<double>("republish_period_sec", 2.0);
    segment_max_waypoints_ = declare_parameter<int>("segment_max_waypoints", 30);
    segment_max_retries_ = declare_parameter<int>("segment_max_retries", 1);
    segment_timeout_sec_ = declare_parameter<double>("segment_timeout_sec", 180.0);
    continue_after_blocked_ = declare_parameter<bool>("continue_after_blocked", false);
    action_name_ = declare_parameter<std::string>(
      "navigate_through_poses_action", "navigate_through_poses");
    if (path_pose_spacing_ <= 0.0 || nav_waypoint_spacing_ <= 0.0 ||
      republish_period_sec_ <= 0.0 || segment_max_waypoints_ < 2 ||
      segment_max_retries_ < 0 || segment_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "spacing/period/timeout must be positive, segment_max_waypoints >= 2, "
              "and segment_max_retries >= 0");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/coverage_path", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/coverage_markers", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/coverage_planner/status", qos);
    nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, action_name_);

    replan_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage_planner/replan",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string message;
        response->success = plan(message);
        response->message = message;
      });

    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage_planner/execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        execute(response->success, response->message);
      });

    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/coverage_planner/cancel",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancel_mission(response->success, response->message);
      });

    mission_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        if (!goal_active_ || cancel_requested_ || !mission_active_) {
          return;
        }
        const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - segment_started_).count();
        if (elapsed <= segment_timeout_sec_) {
          return;
        }
        cancel_requested_ = true;
        RCLCPP_WARN(
          get_logger(), "coverage segment %zu timed out after %.1f seconds; canceling Nav2 goal",
          current_segment_, segment_timeout_sec_);
        if (active_goal_handle_) {
          nav_client_->async_cancel_goal(active_goal_handle_);
        }
      });

    republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(republish_period_sec_)),
      [this]() {
        if (!path_.poses.empty()) {
          const auto stamp = now();
          path_.header.stamp = stamp;
          for (auto & pose : path_.poses) {
            pose.header.stamp = stamp;
          }
          for (auto & marker : markers_.markers) {
            marker.header.stamp = stamp;
          }
          path_pub_->publish(path_);
          marker_pub_->publish(markers_);
        }
      });

    if (auto_plan_) {
      startup_timer_ = create_wall_timer(
        std::chrono::milliseconds(250), [this]() {
          startup_timer_->cancel();
          std::string message;
          if (!plan(message)) {
            RCLCPP_ERROR(get_logger(), "%s", message.c_str());
          }
        });
    }
  }

private:
  AreaConfig load_area_config() const
  {
    if (area_file_.empty()) {
      throw std::runtime_error("area_file parameter is empty");
    }
    const YAML::Node root = YAML::LoadFile(area_file_);
    AreaConfig config;
    if (root["frame_id"]) {
      config.frame_id = root["frame_id"].as<std::string>();
    }
    config.boundary = read_ring(root["boundary"], "boundary");
    orient_ring(config.boundary, true);

    if (root["exclusions"]) {
      if (!root["exclusions"].IsSequence()) {
        throw std::runtime_error("exclusions must be a sequence of polygon rings");
      }
      for (size_t i = 0; i < root["exclusions"].size(); ++i) {
        auto ring = read_ring(root["exclusions"][i], "exclusions[" + std::to_string(i) + "]");
        orient_ring(ring, false);
        config.exclusions.push_back(std::move(ring));
      }
    }

    const auto robot = root["robot"];
    if (robot) {
      if (robot["width"]) {config.robot_width = robot["width"].as<double>();}
      if (robot["coverage_width"]) {
        config.coverage_width = robot["coverage_width"].as<double>();
      }
      if (robot["min_turning_radius"]) {
        config.min_turning_radius = robot["min_turning_radius"].as<double>();
      }
      if (robot["cruise_speed"]) {
        config.cruise_speed = robot["cruise_speed"].as<double>();
      }
      if (robot["turn_speed"]) {config.turn_speed = robot["turn_speed"].as<double>();}
    }

    const auto planner = root["planner"];
    if (planner) {
      if (planner["headland_swaths"]) {
        config.headland_swaths = planner["headland_swaths"].as<int>();
      }
      if (planner["swath_angle_deg"] && !planner["swath_angle_deg"].IsNull()) {
        config.use_fixed_swath_angle = true;
        config.swath_angle_rad = planner["swath_angle_deg"].as<double>() * M_PI / 180.0;
      }
    }

    if (config.frame_id.empty()) {
      throw std::runtime_error("frame_id must not be empty");
    }
    if (config.robot_width <= 0.0 || config.coverage_width <= 0.0 ||
      config.min_turning_radius <= 0.0)
    {
      throw std::runtime_error("robot width, coverage_width, and min_turning_radius must be positive");
    }
    if (config.headland_swaths < 1) {
      throw std::runtime_error("planner.headland_swaths must be at least 1");
    }
    return config;
  }

  F2CCell make_cell(const AreaConfig & config) const
  {
    F2CLinearRing outer(config.boundary);
    outer.closeRing();
    F2CCell cell(outer);
    for (const auto & exclusion : config.exclusions) {
      F2CLinearRing hole(exclusion);
      hole.closeRing();
      cell.addRing(hole);
    }
    if (cell.area() <= 0.0) {
      throw std::runtime_error("work area is empty after applying exclusions");
    }
    return cell;
  }

  void validate_path(const F2CPath & path, const AreaConfig & config) const
  {
    size_t outside_boundary = 0;
    size_t inside_exclusion = 0;
    for (const auto & state : path.getStates()) {
      const double x = state.point.getX();
      const double y = state.point.getY();
      if (!point_in_polygon(x, y, config.boundary)) {
        ++outside_boundary;
      }
      for (const auto & exclusion : config.exclusions) {
        if (point_in_polygon(x, y, exclusion)) {
          ++inside_exclusion;
          break;
        }
      }
    }
    if (outside_boundary > 0 || inside_exclusion > 0) {
      throw std::runtime_error(
              "generated path failed safety validation: " +
              std::to_string(outside_boundary) + " states outside boundary, " +
              std::to_string(inside_exclusion) + " states inside exclusions; " +
              "increase headland_swaths, enlarge exclusion polygons, or change the work area");
    }
  }

  void validate_planning_feasibility(
    const F2CCell & cell, const F2CRobot & robot, const f2c::Options & options) const
  {
    if (options.hg_alg == f2c::HGAlg::NONE) {
      return;
    }

    const F2CCells cells(cell);
    f2c::hg::ConstHL headland_generator;
    const auto mainland = headland_generator.generateHeadlandArea(
      cells, robot.getWidth(), options.hg_swaths);
    if (mainland.size() == 0 || mainland.area() <= 0.0) {
      throw std::runtime_error(
              "no mainland remains after reserving headlands; reduce headland_swaths, "
              "enlarge the boundary, or simplify exclusions");
    }

    const auto headland_swaths = headland_generator.generateHeadlandSwaths(
      cells, robot.getWidth(), options.hg_swaths);
    const size_t selected_headland = static_cast<size_t>(std::floor(0.5 * options.hg_swaths));
    if (selected_headland >= headland_swaths.size() ||
      headland_swaths[selected_headland].size() == 0)
    {
      throw std::runtime_error(
              "no valid headland route remains; reduce headland_swaths or enlarge clearances");
    }

    f2c::sg::BruteForce swath_generator;
    F2CSwathsByCells swaths;
    if (options.sg_alg == f2c::SGAlg::GIVEN_ANGLE) {
      swaths = swath_generator.generateSwaths(
        options.sg_angle, robot.getCovWidth(), mainland);
    } else {
      f2c::obj::NSwathModified objective;
      swaths = swath_generator.generateBestSwaths(
        objective, robot.getCovWidth(), mainland);
    }
    if (swaths.sizeTotal() == 0) {
      throw std::runtime_error(
              "no mowing swaths remain after applying headlands and exclusions; "
              "reduce headland_swaths, enlarge the boundary, or simplify exclusions");
    }
  }

  geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double yaw, const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
  }

  nav_msgs::msg::Path make_path_message(
    const F2CPath & path, const AreaConfig & config, const rclcpp::Time & stamp,
    std::vector<size_t> & state_indices) const
  {
    nav_msgs::msg::Path message;
    message.header.stamp = stamp;
    message.header.frame_id = config.frame_id;
    message.poses.reserve(path.size() + 1);
    state_indices.clear();
    double previous_x = 0.0;
    double previous_y = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
      const auto & state = path.getState(i);
      const bool first = message.poses.empty();
      const bool section_changed =
        i > 0 && state.type != path.getState(i - 1).type;
      const double distance = first ? 0.0 : std::hypot(
        state.point.getX() - previous_x, state.point.getY() - previous_y);
      if (!first && !section_changed && distance < path_pose_spacing_) {
        continue;
      }
      message.poses.push_back(make_pose(
        state.point.getX(), state.point.getY(), state.angle, stamp, config.frame_id));
      state_indices.push_back(i);
      previous_x = state.point.getX();
      previous_y = state.point.getY();
    }
    if (path.size() > 0) {
      const auto & last = path.back();
      const auto end = last.atEnd();
      message.poses.push_back(make_pose(
        end.getX(), end.getY(), last.angle, stamp, config.frame_id));
      state_indices.push_back(path.size());
    }
    return message;
  }

  visualization_msgs::msg::Marker make_ring_marker(
    const std::vector<F2CPoint> & ring, int id, const std::string & ns,
    float red, float green, float blue, const AreaConfig & config,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = config.frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.10;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;
    for (const auto & point : ring) {
      geometry_msgs::msg::Point p;
      p.x = point.getX();
      p.y = point.getY();
      p.z = 0.05;
      marker.points.push_back(p);
    }
    marker.points.push_back(marker.points.front());
    return marker;
  }

  visualization_msgs::msg::MarkerArray make_markers(
    const AreaConfig & config, const nav_msgs::msg::Path & path,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    markers.markers.push_back(make_ring_marker(
      config.boundary, 0, "work_boundary", 0.10F, 0.85F, 0.25F, config, stamp));
    for (size_t i = 0; i < config.exclusions.size(); ++i) {
      markers.markers.push_back(make_ring_marker(
        config.exclusions[i], static_cast<int>(i), "exclusion", 0.95F, 0.20F, 0.15F,
        config, stamp));
    }

    visualization_msgs::msg::Marker path_marker;
    path_marker.header = path.header;
    path_marker.ns = "coverage_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.07;
    path_marker.color.r = 0.10F;
    path_marker.color.g = 0.55F;
    path_marker.color.b = 1.0F;
    path_marker.color.a = 1.0F;
    for (const auto & pose : path.poses) {
      auto point = pose.pose.position;
      point.z = 0.08;
      path_marker.points.push_back(point);
    }
    markers.markers.push_back(std::move(path_marker));
    return markers;
  }

  std::vector<NavWaypoint> make_nav_waypoints(
    const nav_msgs::msg::Path & path, const F2CPath & f2c_path,
    const std::vector<size_t> & state_indices) const
  {
    std::vector<NavWaypoint> waypoints;
    if (path.poses.empty()) {
      return waypoints;
    }
    const auto path_type_at = [&f2c_path, &state_indices](size_t index) {
        const size_t state_index = state_indices[index];
        if (state_index < f2c_path.size()) {
          return f2c_path.getState(state_index).type;
        }
        return f2c_path.back().type;
      };

    waypoints.push_back({path.poses.front(), path_type_at(0)});
    double accumulated = 0.0;
    auto previous = path.poses.front().pose.position;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & current = path.poses[i].pose.position;
      accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
      const auto type = path_type_at(i);
      const bool section_changed = type != path_type_at(i - 1);
      if (section_changed || accumulated >= nav_waypoint_spacing_) {
        waypoints.push_back({path.poses[i], type});
        accumulated = 0.0;
      }
      previous = current;
    }
    if (waypoints.back().pose.pose.position.x != path.poses.back().pose.position.x ||
      waypoints.back().pose.pose.position.y != path.poses.back().pose.position.y)
    {
      waypoints.push_back({path.poses.back(), path_type_at(path.poses.size() - 1)});
    }
    return waypoints;
  }

  std::vector<CoverageSegment> make_segments(
    const std::vector<NavWaypoint> & waypoints)
  {
    std::vector<CoverageSegment> segments;
    if (waypoints.empty()) {
      return segments;
    }
    CoverageSegment segment;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const bool starts_work_swath =
        i > 0 && waypoints[i].type != f2c::types::PathSectionType::TURN &&
        waypoints[i - 1].type == f2c::types::PathSectionType::TURN;
      if (starts_work_swath && !segment.poses.empty()) {
        if (segment.poses.size() > static_cast<size_t>(segment_max_waypoints_)) {
          RCLCPP_WARN(
            get_logger(), "coverage segment %zu has %zu waypoints, above advisory limit %d; "
            "keeping the complete swath/turn sequence intact",
            segments.size() + 1, segment.poses.size(), segment_max_waypoints_);
        }
        segments.push_back(std::move(segment));
        segment = CoverageSegment{};
      }
      segment.poses.push_back(waypoints[i].pose);
    }
    if (!segment.poses.empty()) {
      if (segment.poses.size() > static_cast<size_t>(segment_max_waypoints_)) {
        RCLCPP_WARN(
          get_logger(), "coverage segment %zu has %zu waypoints, above advisory limit %d; "
          "keeping the complete swath/turn sequence intact",
          segments.size() + 1, segment.poses.size(), segment_max_waypoints_);
      }
      segments.push_back(std::move(segment));
    }
    return segments;
  }

  void publish_status(const std::string & event = "")
  {
    size_t pending = 0;
    size_t active = 0;
    size_t completed = 0;
    size_t blocked = 0;
    size_t canceled = 0;
    for (const auto & segment : segments_) {
      switch (segment.state) {
        case SegmentState::PENDING: ++pending; break;
        case SegmentState::ACTIVE: ++active; break;
        case SegmentState::COMPLETED: ++completed; break;
        case SegmentState::BLOCKED: ++blocked; break;
        case SegmentState::CANCELED: ++canceled; break;
      }
    }
    std_msgs::msg::String status;
    status.data =
      "mission_active=" + std::string(mission_active_ ? "true" : "false") +
      " current_segment=" + std::to_string(current_segment_) +
      " total=" + std::to_string(segments_.size()) +
      " pending=" + std::to_string(pending) +
      " active=" + std::to_string(active) +
      " completed=" + std::to_string(completed) +
      " blocked=" + std::to_string(blocked) +
      " canceled=" + std::to_string(canceled);
    if (!event.empty()) {
      status.data += " event=" + event;
    }
    status_pub_->publish(status);
    save_mission_state(event);
  }

  void save_mission_state(const std::string & event)
  {
    if (mission_state_file_.empty()) {
      return;
    }
    try {
      const std::filesystem::path output(expand_user_path(mission_state_file_));
      if (output.has_parent_path()) {
        std::filesystem::create_directories(output.parent_path());
      }
      YAML::Emitter yaml;
      yaml << YAML::BeginMap;
      yaml << YAML::Key << "source_area_file" << YAML::Value << area_file_;
      yaml << YAML::Key << "event" << YAML::Value << event;
      yaml << YAML::Key << "mission_active" << YAML::Value << mission_active_;
      yaml << YAML::Key << "current_segment_index" << YAML::Value << current_segment_;
      yaml << YAML::Key << "segments" << YAML::Value << YAML::BeginSeq;
      for (const auto & segment : segments_) {
        yaml << YAML::Flow << YAML::BeginMap;
        yaml << YAML::Key << "state" << YAML::Value <<
          (segment.state == SegmentState::PENDING ? "PENDING" :
          segment.state == SegmentState::ACTIVE ? "ACTIVE" :
          segment.state == SegmentState::COMPLETED ? "COMPLETED" :
          segment.state == SegmentState::BLOCKED ? "BLOCKED" : "CANCELED");
        yaml << YAML::Key << "attempts" << YAML::Value << segment.attempts;
        yaml << YAML::Key << "waypoint_count" << YAML::Value << segment.poses.size();
        yaml << YAML::EndMap;
      }
      yaml << YAML::EndSeq << YAML::EndMap;
      std::ofstream stream(output);
      if (!stream) {
        throw std::runtime_error("cannot open mission state file " + output.string());
      }
      stream << yaml.c_str() << '\n';
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "failed to save coverage mission state: %s", exception.what());
    }
  }

  void save_path(
    const std::string & file, const AreaConfig & config, const F2CPath & f2c_path,
    const nav_msgs::msg::Path & path, const std::vector<size_t> & state_indices) const
  {
    if (file.empty()) {
      return;
    }
    const std::filesystem::path output(expand_user_path(file));
    if (output.has_parent_path()) {
      std::filesystem::create_directories(output.parent_path());
    }

    YAML::Emitter yaml;
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "frame_id" << YAML::Value << config.frame_id;
    yaml << YAML::Key << "source_area_file" << YAML::Value << area_file_;
    yaml << YAML::Key << "path_length_m" << YAML::Value << f2c_path.length();
    yaml << YAML::Key << "poses" << YAML::Value << YAML::BeginSeq;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & pose = path.poses[i];
      const double yaw = 2.0 * std::atan2(pose.pose.orientation.z, pose.pose.orientation.w);
      yaml << YAML::Flow << YAML::BeginMap;
      yaml << YAML::Key << "x" << YAML::Value << pose.pose.position.x;
      yaml << YAML::Key << "y" << YAML::Value << pose.pose.position.y;
      yaml << YAML::Key << "yaw" << YAML::Value << yaw;
      if (state_indices[i] < f2c_path.size()) {
        const auto & state = f2c_path.getState(state_indices[i]);
        yaml << YAML::Key << "section" << YAML::Value <<
          (state.type == f2c::types::PathSectionType::TURN ? "turn" : "swath");
        yaml << YAML::Key << "direction" << YAML::Value <<
          (state.dir == f2c::types::PathDirection::BACKWARD ? "backward" : "forward");
      }
      yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq << YAML::EndMap;

    std::ofstream stream(output);
    if (!stream) {
      throw std::runtime_error("cannot open output file " + output.string());
    }
    stream << yaml.c_str() << '\n';
  }

  bool plan(std::string & message)
  {
    if (mission_active_ || goal_active_) {
      message = "coverage planning failed: cancel the active mission before replanning";
      return false;
    }
    try {
      const auto config = load_area_config();
      auto cell = make_cell(config);
      F2CRobot robot(config.robot_width, config.coverage_width);
      robot.setMinTurningRadius(config.min_turning_radius);
      robot.setCruiseVel(config.cruise_speed);
      robot.setTurnVel(config.turn_speed);

      f2c::Options options;
      options.hg_swaths = config.headland_swaths;
      if (config.use_fixed_swath_angle) {
        options.sg_alg = f2c::SGAlg::GIVEN_ANGLE;
        options.sg_angle = config.swath_angle_rad;
      }

      validate_planning_feasibility(cell, robot, options);
      const auto f2c_path = f2c::planCovPath(robot, cell, options);
      if (f2c_path.size() == 0) {
        throw std::runtime_error(
                "Fields2Cover returned an empty path; enlarge the area or reduce headland/robot dimensions");
      }
      validate_path(f2c_path, config);

      const auto stamp = now();
      std::vector<size_t> state_indices;
      auto path = make_path_message(f2c_path, config, stamp, state_indices);
      auto markers = make_markers(config, path, stamp);
      auto typed_nav_waypoints = make_nav_waypoints(path, f2c_path, state_indices);
      std::vector<geometry_msgs::msg::PoseStamped> nav_waypoints;
      nav_waypoints.reserve(typed_nav_waypoints.size());
      for (const auto & waypoint : typed_nav_waypoints) {
        nav_waypoints.push_back(waypoint.pose);
      }
      save_path(output_file_, config, f2c_path, path, state_indices);

      config_ = config;
      path_ = std::move(path);
      markers_ = std::move(markers);
      nav_waypoints_ = std::move(nav_waypoints);
      segments_ = make_segments(typed_nav_waypoints);
      current_segment_ = 0;
      path_pub_->publish(path_);
      marker_pub_->publish(markers_);

      message = "planned " + std::to_string(path_.poses.size()) + " path poses and " +
        std::to_string(nav_waypoints_.size()) + " Nav2 waypoints in " +
        std::to_string(segments_.size()) + " segments; length=" +
        std::to_string(f2c_path.length()) + " m";
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      if (dry_run_) {
        RCLCPP_INFO(
          get_logger(), "dry_run=true: path is published but will not be sent to Nav2");
      }
      publish_status("planned");
      return true;
    } catch (const std::exception & exception) {
      message = std::string("coverage planning failed: ") + exception.what();
      return false;
    }
  }

  void execute(bool & success, std::string & message)
  {
    if (dry_run_) {
      success = false;
      message = "dry_run=true; set dry_run:=false before executing a Nav2 goal";
      return;
    }
    if (mission_active_ || goal_active_) {
      success = false;
      message = "a coverage navigation goal is already active";
      return;
    }
    if (segments_.empty()) {
      success = false;
      message = "no path is available; call /coverage_planner/replan first";
      return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
      success = false;
      message = "Nav2 action server " + action_name_ + " is unavailable";
      return;
    }

    for (auto & segment : segments_) {
      segment.state = SegmentState::PENDING;
      segment.attempts = 0;
    }
    current_segment_ = 0;
    mission_active_ = true;
    user_canceling_ = false;
    submit_current_segment();
    success = true;
    message = "started coverage mission with " + std::to_string(segments_.size()) + " segments";
  }

  void submit_current_segment()
  {
    while (current_segment_ < segments_.size() &&
      segments_[current_segment_].state != SegmentState::PENDING)
    {
      ++current_segment_;
    }
    if (current_segment_ >= segments_.size()) {
      mission_active_ = false;
      goal_active_ = false;
      active_goal_handle_.reset();
      publish_status("mission_finished");
      RCLCPP_INFO(get_logger(), "coverage mission finished");
      return;
    }

    auto & segment = segments_[current_segment_];
    segment.state = SegmentState::ACTIVE;
    ++segment.attempts;
    NavigateThroughPoses::Goal goal;
    goal.poses = segment.poses;
    for (auto & pose : goal.poses) {
      pose.header.stamp = now();
    }

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
    const size_t submitted_segment = current_segment_;
    options.goal_response_callback = [this, submitted_segment](
      GoalHandleNavigateThroughPoses::SharedPtr handle) {
        if (submitted_segment != current_segment_ || !mission_active_) {
          return;
        }
        if (!handle) {
          goal_active_ = false;
          if (user_canceling_) {
            segments_[current_segment_].state = SegmentState::CANCELED;
            mission_active_ = false;
            user_canceling_ = false;
            publish_status("mission_canceled");
            return;
          }
          handle_segment_failure("goal_rejected");
          return;
        }
        active_goal_handle_ = handle;
        if (cancel_requested_) {
          nav_client_->async_cancel_goal(active_goal_handle_);
          return;
        }
        RCLCPP_INFO(
          get_logger(), "Nav2 accepted coverage segment %zu/%zu (attempt %d)",
          current_segment_ + 1, segments_.size(), segments_[current_segment_].attempts);
      };
    options.result_callback = [this, submitted_segment](
      const GoalHandleNavigateThroughPoses::WrappedResult & result) {
        if (submitted_segment != current_segment_ || !mission_active_) {
          return;
        }
        goal_active_ = false;
        active_goal_handle_.reset();
        const bool timed_out = cancel_requested_;
        cancel_requested_ = false;
        if (user_canceling_) {
          segments_[current_segment_].state = SegmentState::CANCELED;
          mission_active_ = false;
          user_canceling_ = false;
          publish_status("mission_canceled");
          return;
        }
        if (timed_out) {
          handle_segment_failure("segment_timeout");
        } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          segments_[current_segment_].state = SegmentState::COMPLETED;
          publish_status("segment_completed");
          ++current_segment_;
          submit_current_segment();
        } else {
          handle_segment_failure("navigation_failed");
        }
      };
    goal_active_ = true;
    cancel_requested_ = false;
    segment_started_ = std::chrono::steady_clock::now();
    publish_status("segment_started");
    nav_client_->async_send_goal(goal, options);
  }

  void handle_segment_failure(const std::string & reason)
  {
    auto & segment = segments_[current_segment_];
    if (segment.attempts <= segment_max_retries_) {
      segment.state = SegmentState::PENDING;
      RCLCPP_WARN(
        get_logger(), "coverage segment %zu failed (%s); retrying (%d/%d)",
        current_segment_ + 1, reason.c_str(), segment.attempts, segment_max_retries_ + 1);
      publish_status("segment_retry");
      submit_current_segment();
      return;
    }
    segment.state = SegmentState::BLOCKED;
    RCLCPP_ERROR(
      get_logger(), "coverage segment %zu blocked after %d attempts (%s)",
      current_segment_ + 1, segment.attempts, reason.c_str());
    publish_status("segment_blocked");
    if (!continue_after_blocked_) {
      mission_active_ = false;
      publish_status("mission_stopped_on_blocked");
      return;
    }
    ++current_segment_;
    submit_current_segment();
  }

  void cancel_mission(bool & success, std::string & message)
  {
    if (!mission_active_) {
      success = false;
      message = "no coverage mission is active";
      return;
    }
    user_canceling_ = true;
    cancel_requested_ = true;
    for (size_t i = current_segment_ + 1; i < segments_.size(); ++i) {
      if (segments_[i].state == SegmentState::PENDING) {
        segments_[i].state = SegmentState::CANCELED;
      }
    }
    if (goal_active_) {
      if (active_goal_handle_) {
        nav_client_->async_cancel_goal(active_goal_handle_);
      }
      success = true;
      message = "cancel requested for active coverage segment";
    } else {
      segments_[current_segment_].state = SegmentState::CANCELED;
      mission_active_ = false;
      user_canceling_ = false;
      success = true;
      message = "coverage mission canceled";
      publish_status("mission_canceled");
    }
  }

  std::string area_file_;
  std::string output_file_;
  std::string mission_state_file_;
  std::string action_name_;
  bool dry_run_{true};
  bool auto_plan_{true};
  bool goal_active_{false};
  bool mission_active_{false};
  bool cancel_requested_{false};
  bool user_canceling_{false};
  bool continue_after_blocked_{false};
  int segment_max_waypoints_{30};
  int segment_max_retries_{1};
  double segment_timeout_sec_{180.0};
  size_t current_segment_{0};
  double path_pose_spacing_{0.10};
  double nav_waypoint_spacing_{0.75};
  double republish_period_sec_{2.0};
  AreaConfig config_;
  nav_msgs::msg::Path path_;
  visualization_msgs::msg::MarkerArray markers_;
  std::vector<geometry_msgs::msg::PoseStamped> nav_waypoints_;
  std::vector<CoverageSegment> segments_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
  GoalHandleNavigateThroughPoses::SharedPtr active_goal_handle_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  std::chrono::steady_clock::time_point segment_started_{};
};

}  // namespace golf_mower_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<golf_mower_bringup::CoveragePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
