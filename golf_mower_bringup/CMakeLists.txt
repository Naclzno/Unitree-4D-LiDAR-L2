cmake_minimum_required(VERSION 3.8)
project(golf_mower_bringup)

find_package(ament_cmake REQUIRED)
find_package(Fields2Cover CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(coverage_planner_node
  src/coverage_planner_node.cpp
)
ament_target_dependencies(coverage_planner_node
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  visualization_msgs
)
target_link_libraries(coverage_planner_node
  Fields2Cover::Fields2Cover
  yaml-cpp
)

add_library(golf_mower_costmap_plugins SHARED
  src/geofence_layer.cpp
  src/traversability_layer.cpp
)
target_include_directories(golf_mower_costmap_plugins PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(golf_mower_costmap_plugins
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
)
target_link_libraries(golf_mower_costmap_plugins yaml-cpp)
pluginlib_export_plugin_description_file(
  nav2_costmap_2d golf_mower_costmap_plugins.xml
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS golf_mower_costmap_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS coverage_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pointlio_odom_tf_bridge.py
  scripts/nav2_send_spin_goal.py
  scripts/nav2_send_spin_action.py
  scripts/nav2_send_tb3_forward_goal.py
  scripts/grid_map_to_occupancy_grid.py
  scripts/grid_map_inspect.py
  scripts/pointcloud_ground_fallback.py
  scripts/pointcloud_xyz_stats.py
  scripts/pointlio_input_diagnostics.py
  scripts/stage5_runtime_diagnostics.py
  scripts/static_pointlio_odom.py
  scripts/pcd_to_occupancy_grid.py
  scripts/coverage_boundary_from_pcd.py
  scripts/segmented_map_builder.py
  scripts/fake_rtk_from_odom.py
  scripts/map_metadata_recorder.py
  scripts/rtk_map_localizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(golf_mower_costmap_plugins)
ament_export_dependencies(nav2_costmap_2d nav_msgs pluginlib rclcpp)

ament_package()
