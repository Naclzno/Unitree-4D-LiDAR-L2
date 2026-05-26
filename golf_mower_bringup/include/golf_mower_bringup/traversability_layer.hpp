#ifndef GOLF_MOWER_BRINGUP__TRAVERSABILITY_LAYER_HPP_
#define GOLF_MOWER_BRINGUP__TRAVERSABILITY_LAYER_HPP_

#include <mutex>
#include <string>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace golf_mower_bringup
{

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer() = default;
  ~TraversabilityLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

private:
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  unsigned char occupancyToCost(int8_t occupancy) const;
  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double wx, double wy, unsigned int & mx, unsigned int & my) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
  std::mutex grid_mutex_;

  std::string map_topic_;
  bool clear_on_no_map_{false};
  bool use_maximum_{true};
  int lethal_threshold_{90};
  int free_threshold_{0};
  double stale_timeout_{2.0};
  rclcpp::Time last_map_time_;
};

}  // namespace golf_mower_bringup

#endif  // GOLF_MOWER_BRINGUP__TRAVERSABILITY_LAYER_HPP_
