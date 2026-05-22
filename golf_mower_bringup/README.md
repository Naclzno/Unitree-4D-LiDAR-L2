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

For serial Unitree L2 testing, use:

```bash
ros2 launch golf_mower_bringup indoor_slam_test.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  launch_rviz:=true
```

The first test chain is:

```text
Unitree L2 -> /unilidar/cloud, /unilidar/imu -> Point-LIO -> /pointlio/odom, /pointlio/laser_map -> RViz
```

The bringup RViz config displays both the raw Unitree cloud `/unilidar/cloud`
and the Point-LIO outputs such as `/pointlio/cloud_registered` and
`/pointlio/laser_map`.

Temporary TF adapter:

```text
map -> camera_init -> aft_mapped -> base_link
                                      -> unilidar_imu
                                      -> unilidar_lidar
```

Point-LIO publishes `camera_init -> aft_mapped`. The Unitree lidar node publishes
raw sensor messages in `unilidar_imu` and `unilidar_lidar`. This launch disables
the Unitree vendor demo TFs and adds `map -> camera_init`, `aft_mapped -> base_link`,
`base_link -> unilidar_imu`, and `base_link -> unilidar_lidar` for indoor bench
testing and later Nav2 integration tests.

## Indoor Nav2 Mock

The second-stage launch keeps the Unitree L2 and Point-LIO chain and starts the
Nav2 navigation stack without AMCL or a 2D map server. Nav2 consumes the SLAM TF
tree; it does not add a new TF branch by itself. The optional odometry-to-TF
bridge is disabled by default because Point-LIO already publishes
`camera_init -> aft_mapped`.

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
                                      -> unilidar_imu
                                      -> unilidar_lidar
```

This launch is for integration checking while the lidar is on the desk. It can plan and publish `/cmd_vel`, but there is no real base driver in the loop yet.

## Nav2 Smoke Demo

After `indoor_nav2_mock.launch.py` is active, send a small in-place yaw goal:

```bash
ros2 run golf_mower_bringup nav2_send_spin_goal.py
```

In another terminal, watch Nav2 controller output:

```bash
ros2 topic echo /cmd_vel
```

Because no real base driver is connected in this desk test, the robot pose will not
actually change. The useful signal is that Nav2 accepts the goal and publishes
`/cmd_vel`; the goal may later time out or fail progress checking.

## TurtleBot3 Gazebo Nav2 Demo

For a full Nav2 movement test, use the official TurtleBot3 Gazebo demo:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

After Gazebo, RViz, and Nav2 are active, send a short forward navigation goal:

```bash
ros2 run golf_mower_bringup nav2_send_tb3_forward_goal.py
```

Watch the velocity command if needed:

```bash
ros2 topic echo /cmd_vel
```

Unlike the indoor desk test, this demo has a simulated robot base, odometry, laser,
and Gazebo physics, so the TurtleBot3 should move when Nav2 publishes `/cmd_vel`.
