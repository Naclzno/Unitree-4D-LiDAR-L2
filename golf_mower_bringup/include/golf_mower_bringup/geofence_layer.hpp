#ifndef GOLF_MOWER_BRINGUP__GEOFENCE_LAYER_HPP_
#define GOLF_MOWER_BRINGUP__GEOFENCE_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace golf_mower_bringup
{

class GeofenceLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  using Point = std::pair<double, double>;

  GeofenceLayer() = default;
  ~GeofenceLayer() override = default;

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
  void loadAreaFile();
  bool insidePolygon(double x, double y, const std::vector<Point> & polygon) const;

  std::string area_file_;
  std::vector<Point> boundary_;
  std::vector<std::vector<Point>> exclusions_;
};

}  // namespace golf_mower_bringup

#endif  // GOLF_MOWER_BRINGUP__GEOFENCE_LAYER_HPP_
