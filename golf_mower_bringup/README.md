# golf_mower_bringup

First-stage bringup package for indoor Unitree L2 and Point-LIO testing.

## Indoor SLAM Test

Build this package after sourcing the already built dependency workspaces:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source unitree_lidar_ros2/install/setup.bash
source point_lio_unilidar-2.0.2/install/setup.bash
colcon build --packages-select golf_mower_bringup --install-base golf_mower_bringup/install --build-base golf_mower_bringup/build --log-base golf_mower_bringup/log
source golf_mower_bringup/install/setup.bash
ros2 launch golf_mower_bringup indoor_slam_test.launch.py
```

The first test chain is:

```text
Unitree L2 -> /unilidar/cloud, /unilidar/imu -> Point-LIO -> /pointlio/odom, /pointlio/laser_map -> RViz
```

Temporary TF adapter:

```text
map -> camera_init -> aft_mapped -> base_link
```

Point-LIO publishes `camera_init -> aft_mapped`; this launch adds `map -> camera_init` and `aft_mapped -> base_link` for later Nav2 integration tests.

## Indoor Nav2 Mock

The second-stage launch keeps the Unitree L2 and Point-LIO chain, adds a small odometry-to-TF bridge, and starts the Nav2 navigation stack without AMCL or a 2D map server.

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source unitree_lidar_ros2/install/setup.bash
source point_lio_unilidar-2.0.2/install/setup.bash
source golf_mower_bringup/install/setup.bash
ros2 launch golf_mower_bringup indoor_nav2_mock.launch.py
```

Expected TF chain:

```text
map -> camera_init -> aft_mapped -> base_link
```

This launch is for integration checking while the lidar is on the desk. It can plan and publish `/cmd_vel`, but there is no real base driver in the loop yet.
