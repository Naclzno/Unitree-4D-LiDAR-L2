#include "golf_mower_bringup/geofence_layer.hpp"

#include <algorithm>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace golf_mower_bringup
{

namespace
{

std::vector<GeofenceLayer::Point> readRing(const YAML::Node & node, const std::string & name)
{
  if (!node || !node.IsSequence() || node.size() < 3) {
    throw std::runtime_error(name + " must contain at least three [x, y] points");
  }
  std::vector<GeofenceLayer::Point> ring;
  ring.reserve(node.size());
  for (size_t i = 0; i < node.size(); ++i) {
    if (!node[i].IsSequence() || node[i].size() < 2) {
      throw std::runtime_error(name + " point " + std::to_string(i) + " must be [x, y]");
    }
    ring.emplace_back(node[i][0].as<double>(), node[i][1].as<double>());
  }
  if (ring.size() > 3 && ring.front() == ring.back()) {
    ring.pop_back();
  }
  return ring;
}

}  // namespace

void GeofenceLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GeofenceLayer failed to lock lifecycle node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("area_file", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".area_file", area_file_);
  if (!enabled_) {
    current_ = true;
    return;
  }
  loadAreaFile();
  current_ = true;
  RCLCPP_INFO(logger_, "GeofenceLayer loaded %s", area_file_.c_str());
}

void GeofenceLayer::loadAreaFile()
{
  if (area_file_.empty()) {
    throw std::runtime_error("GeofenceLayer requires area_file when enabled");
  }
  const YAML::Node root = YAML::LoadFile(area_file_);
  boundary_ = readRing(root["boundary"], "boundary");
  exclusions_.clear();
  if (root["exclusions"]) {
    if (!root["exclusions"].IsSequence()) {
      throw std::runtime_error("exclusions must be a sequence of polygon rings");
    }
    for (size_t i = 0; i < root["exclusions"].size(); ++i) {
      exclusions_.push_back(readRing(root["exclusions"][i], "exclusions[" + std::to_string(i) + "]"));
    }
  }
}

bool GeofenceLayer::insidePolygon(
  double x, double y, const std::vector<Point> & polygon) const
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    const bool crosses = ((a.second > y) != (b.second > y)) &&
      (x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

void GeofenceLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || boundary_.empty()) {
    return;
  }
  // The forbidden region includes every cell outside boundary_, so a boundary
  // bounding box is insufficient when the static map extends beyond it.
  const auto * costmap = layered_costmap_->getCostmap();
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();
  const double end_x = origin_x + costmap->getSizeInCellsX() * costmap->getResolution();
  const double end_y = origin_y + costmap->getSizeInCellsY() * costmap->getResolution();
  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, end_x);
  *max_y = std::max(*max_y, end_y);
}

void GeofenceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || boundary_.empty()) {
    return;
  }
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double x = 0.0;
      double y = 0.0;
      master_grid.mapToWorld(i, j, x, y);
      bool forbidden = !insidePolygon(x, y, boundary_);
      if (!forbidden) {
        for (const auto & exclusion : exclusions_) {
          if (insidePolygon(x, y, exclusion)) {
            forbidden = true;
            break;
          }
        }
      }
      if (forbidden) {
        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void GeofenceLayer::reset()
{
  current_ = enabled_ ? !boundary_.empty() : true;
}

bool GeofenceLayer::isClearable()
{
  return false;
}

}  // namespace golf_mower_bringup

PLUGINLIB_EXPORT_CLASS(golf_mower_bringup::GeofenceLayer, nav2_costmap_2d::Layer)
