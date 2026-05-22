cmake_minimum_required(VERSION 3.8)
project(golf_mower_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pointlio_odom_tf_bridge.py
  scripts/nav2_send_spin_goal.py
  scripts/nav2_send_tb3_forward_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
