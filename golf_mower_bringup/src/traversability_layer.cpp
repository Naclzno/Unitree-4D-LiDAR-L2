#include "golf_mower_bringup/traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace golf_mower_bringup
{

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_topic", rclcpp::ParameterValue(std::string("/elevation/traversability_grid")));
  declareParameter("use_maximum", rclcpp::ParameterValue(true));
  declareParameter("clear_on_no_map", rclcpp::ParameterValue(false));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(90));
  declareParameter("free_threshold", rclcpp::ParameterValue(0));
  declareParameter("stale_timeout", rclcpp::ParameterValue(2.0));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".map_topic", map_topic_);
  node->get_parameter(name_ + ".use_maximum", use_maximum_);
  node->get_parameter(name_ + ".clear_on_no_map", clear_on_no_map_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".free_threshold", free_threshold_);
  node->get_parameter(name_ + ".stale_timeout", stale_timeout_);

  lethal_threshold_ = std::clamp(lethal_threshold_, 1, 100);
  free_threshold_ = std::clamp(free_threshold_, 0, lethal_threshold_ - 1);

  last_map_time_ = node->now();
  current_ = false;

  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&TraversabilityLayer::incomingMap, this, std::placeholders::_1),
    rclcpp::SubscriptionOptions());

  RCLCPP_INFO(
    logger_,
    "TraversabilityLayer subscribed to %s, lethal_threshold=%d, free_threshold=%d",
    map_topic_.c_str(), lethal_threshold_, free_threshold_);
}

void TraversabilityLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_grid_ = msg;
  auto node = node_.lock();
  last_map_time_ = node ? node->now() : rclcpp::Clock().now();
  current_ = true;

  const double origin_x = msg->info.origin.position.x;
  const double origin_y = msg->info.origin.position.y;
  const double size_x = msg->info.width * msg->info.resolution;
  const double size_y = msg->info.height * msg->info.resolution;
  addExtraBounds(origin_x, origin_y, origin_x + size_x, origin_y + size_y);
}

void TraversabilityLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  std::lock_guard<std::mutex> lock(grid_mutex_);
  if (!latest_grid_) {
    current_ = !clear_on_no_map_;
    return;
  }

  auto node = node_.lock();
  if (node && stale_timeout_ > 0.0) {
    const double age = (node->now() - last_map_time_).seconds();
    current_ = age <= stale_timeout_;
  }

  const auto & info = latest_grid_->info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double size_x = info.width * info.resolution;
  const double size_y = info.height * info.resolution;

  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, origin_x + size_x);
  *max_y = std::max(*max_y, origin_y + size_y);
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid = latest_grid_;
  }

  if (!grid) {
    return;
  }

  if (grid->header.frame_id != layered_costmap_->getGlobalFrameID()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "TraversabilityLayer grid frame '%s' does not match costmap global frame '%s'",
      grid->header.frame_id.c_str(), layered_costmap_->getGlobalFrameID().c_str());
    return;
  }

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(size_x, max_i);
  max_j = std::min(size_y, max_j);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);

      unsigned int gx = 0;
      unsigned int gy = 0;
      if (!worldToGrid(*grid, wx, wy, gx, gy)) {
        continue;
      }

      const auto index = gy * grid->info.width + gx;
      if (index >= grid->data.size()) {
        continue;
      }

      const unsigned char terrain_cost = occupancyToCost(grid->data[index]);
      if (terrain_cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      const unsigned char old_cost = master_grid.getCost(i, j);
      if (use_maximum_) {
        if (old_cost == nav2_costmap_2d::NO_INFORMATION || terrain_cost > old_cost) {
          master_grid.setCost(i, j, terrain_cost);
        }
      } else if (terrain_cost > nav2_costmap_2d::FREE_SPACE) {
        master_grid.setCost(i, j, terrain_cost);
      }
    }
  }
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_grid_.reset();
  current_ = false;
}

bool TraversabilityLayer::isClearable()
{
  return false;
}

unsigned char TraversabilityLayer::occupancyToCost(const int8_t occupancy) const
{
  if (occupancy < 0) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  if (occupancy <= free_threshold_) {
    return nav2_costmap_2d::FREE_SPACE;
  }
  if (occupancy >= lethal_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const double normalized =
    static_cast<double>(occupancy - free_threshold_) /
    static_cast<double>(lethal_threshold_ - free_threshold_);
  const auto cost = static_cast<unsigned char>(
    std::round(normalized * (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));
  return std::clamp<unsigned char>(
    cost, nav2_costmap_2d::FREE_SPACE + 1, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
}

bool TraversabilityLayer::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const double wx, const double wy,
  unsigned int & mx, unsigned int & my) const
{
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double resolution = grid.info.resolution;

  if (resolution <= 0.0 || wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);

  return mx < grid.info.width && my < grid.info.height;
}

}  // namespace golf_mower_bringup

PLUGINLIB_EXPORT_CLASS(golf_mower_bringup::TraversabilityLayer, nav2_costmap_2d::Layer)
