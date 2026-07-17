# Golf Mower Robot ROS2 Project

English | [中文](README_CN.md)

This ROS2 source repository contains the algorithm stack for a golf-course mowing robot. It is built around the Unitree L2 lidar, Point-LIO, Patchwork++ ground segmentation, elevation/traversability mapping, Nav2 navigation, and UM981 RTK/INS localization.

The current full outdoor workflow is Stage 5:

```text
Unitree L2
  -> /unilidar/cloud, /unilidar/imu
  -> Point-LIO
  -> /pointlio/odom, /pointlio/cloud_registered, /pointlio/laser_map
  -> Patchwork++ ground/non-ground segmentation
  -> offline segmented PCD map
  -> RTK/INS initial localization in the offline map
  -> Nav2 navigation over the loaded map
```

## Main Capabilities

- Read Unitree L2 point cloud and IMU data over serial or UDP.
- Run Point-LIO for lidar-inertial odometry and local point cloud mapping.
- Segment ground and obstacle points with Patchwork++.
- Save offline segmented maps as `ground_map.pcd` and `nonground_map.pcd`.
- Record RTK georeference metadata for the saved map.
- Load the offline map for Nav2 navigation.
- Use UM981 GNSS `/fix` for map initialization; provide initial yaw manually for now.
- Provide indoor test modes with fake RTK when GNSS/RTK signal is unavailable.
- Generate ROS2 coverage paths from map-frame boundaries and exclusions with Fields2Cover.
- Translate `/cmd_vel` into the documented motor-controller serial protocol with a default-safe, explicitly armed driver.
- Keep earlier Stage 1/2/3/4 launches for incremental debugging.

## Current Project Status

| Subsystem | Status | Current scope |
| --- | --- | --- |
| Unitree L2 point cloud and IMU | Integrated | Serial/UDP driver publishes `/unilidar/cloud` and `/unilidar/imu`. |
| Point-LIO odometry | Integrated | Publishes lidar-inertial odometry and registered point clouds. |
| Patchwork++ segmentation | Integrated, under calibration | Ground/non-ground topics are connected; ground output can still become empty and requires mounting/axis validation. |
| Live elevation and traversability | Integrated | Supports GPU or CPU elevation mapping and a conservative low-surface fallback. |
| Offline segmented mapping | Integrated | Saves timestamped ground, non-ground, and georeference metadata; supports offline ground recovery. |
| UM981 GNSS position | Partially integrated | GGA `/fix` initializes the robot in the offline map; UM981 IMU/INS heading is not used. |
| Continuous localization fusion | Not integrated | RTK currently initializes `map -> camera_init`; wheel odometry, RTK, IMU, and Point-LIO are not yet fused continuously. |
| Nav2 planning and obstacle avoidance | Integrated at algorithm level | Produces `/cmd_vel` using the offline global map and live local terrain/obstacles. |
| Chassis motor driver | Prototype integrated | `motor_driver_node.py` maps the gated motor velocity stream to documented discrete commands, with timeout brake, dry-run, and explicit arming. Serial settings and real-wheel calibration remain unverified. |
| Wheel encoder odometry | Not implemented | No `/wheel/odom` source or calibrated chassis kinematics. |
| Mower actuator | Model only | `mower_tool` exists in URDF, but there is no blade motor interface, feedback, or fault handling. |
| Coverage path planning | Integrated with ROS2 | Reads map-frame boundaries and exclusions from YAML, publishes paths/markers, saves results, and provides an optional Nav2 action; dry-run is the default. |
| Mission manager | Basic implementation | Executes complete swath/turn sequences with retries, timeouts, blocked-segment handling, and cancellation; actual coverage tracking and recovery remain. |
| Safety system | Initial software gate integrated | The safety controller gates `/cmd_vel` using explicit enable, odometry freshness, command freshness, and a latched software E-stop; hardware emergency stop and blade interlock remain. |

The repository currently provides a perception, mapping, localization-initialization, and navigation-planning prototype. It is not yet a complete autonomous mower: physical motion, blade actuation, coverage execution, and safety interlocks must be implemented and validated before field mowing.

The next milestone is physical validation of the chassis loop: motor protocol feedback, wheel encoder odometry, a wired emergency stop, and controlled low-speed tests. Continuous state estimation, physical coverage execution, blade control, and the mission/safety state machine follow after that.

## Directory Layout

| Directory | Purpose |
| --- | --- |
| `unitree_lidar_sdk` | Original Unitree L2 C++ SDK. |
| `unitree_lidar_ros2` | Unitree L2 ROS2 driver, publishing `/unilidar/cloud` and `/unilidar/imu`. |
| `point_lio_unilidar-2.0.2` | Point-LIO adapted for Unitree L2. |
| `patchwork-plusplus-ros` | ROS2 Patchwork++ ground segmentation. |
| `elevation_mapping_cupy` | GPU/CuPy elevation mapping. |
| `elevation_mapping_ros2` | CPU elevation mapping compatibility package. |
| `golf_mower_description` | Robot URDF/Xacro and sensor frames. |
| `golf_mower_bringup` | Main launch files, configuration, diagnostics, map utilities, Nav2 integration, coverage planner, motor driver, and safety controller. |
| `motor` | Motor-controller Serial V2 protocol reference. |
| `UM981` | Python SDK and ROS2 node for UM981 GNSS/RTK/INS. |
| `Fields2Cover-main` | Fields2Cover library used by the ROS2 planner in `golf_mower_bringup`. |
| `robot_localization-rolling-devel` | Reserved for later wheel odometry, RTK, IMU, and lidar odometry fusion. |
| `autoware` | Autoware source tree; not required for the current main workflow. |

## Environment

| Environment component | Version or model |
| --- | --- |
| Operating system | Ubuntu 22.04.5 LTS (Jammy Jellyfish), x86_64 |
| GPU | NVIDIA GeForce RTX 3060 Lite Hash Rate |
| NVIDIA driver | `535.309.01` |
| CUDA | CUDA 12.2 |
| ROS 2 | ROS 2 Humble Hawksbill |

Start each terminal with:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
```

For serial Unitree L2, check the device:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

For UM981, check the device:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Confirmed serial ports on the real Ubuntu robot PC:

Unitree Lidar:

```text
/dev/ttyACM0
```

Stable path:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_593A032669-if00
```

UM981 development board:

```text
/dev/ttyUSB0
```

Stable path:

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

## Build

Build and source the packages in dependency order:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash

colcon --log-base unitree_lidar_ros2/log build \
  --base-paths unitree_lidar_ros2/src \
  --install-base unitree_lidar_ros2/install \
  --build-base unitree_lidar_ros2/build
source unitree_lidar_ros2/install/setup.bash

colcon --log-base point_lio_unilidar-2.0.2/log build \
  --base-paths point_lio_unilidar-2.0.2 \
  --install-base point_lio_unilidar-2.0.2/install \
  --build-base point_lio_unilidar-2.0.2/build
source point_lio_unilidar-2.0.2/install/setup.bash

colcon --log-base patchwork-plusplus-ros/log build \
  --base-paths patchwork-plusplus-ros \
  --install-base patchwork-plusplus-ros/install \
  --build-base patchwork-plusplus-ros/build
source patchwork-plusplus-ros/install/setup.bash

colcon --log-base elevation_mapping_cupy/log build \
  --base-paths elevation_mapping_cupy \
  --install-base elevation_mapping_cupy/install \
  --build-base elevation_mapping_cupy/build
source elevation_mapping_cupy/install/setup.bash

colcon --log-base elevation_mapping_ros2/log build \
  --base-paths elevation_mapping_ros2 \
  --install-base elevation_mapping_ros2/install \
  --build-base elevation_mapping_ros2/build
source elevation_mapping_ros2/install/setup.bash

colcon --log-base golf_mower_description/log build \
  --base-paths golf_mower_description \
  --install-base golf_mower_description/install \
  --build-base golf_mower_description/build
source golf_mower_description/install/setup.bash

colcon --log-base UM981/log build \
  --base-paths UM981 \
  --packages-select um981_ros \
  --install-base UM981/install \
  --build-base UM981/build
source UM981/install/setup.bash

colcon --log-base Fields2Cover-main/log build \
  --base-paths Fields2Cover-main \
  --packages-select fields2cover \
  --install-base Fields2Cover-main/install \
  --build-base Fields2Cover-main/build \
  --cmake-args -DBUILD_TUTORIALS=OFF -DBUILD_PYTHON=OFF -DBUILD_TESTING=OFF
source Fields2Cover-main/install/setup.bash

colcon --log-base golf_mower_bringup/log build \
  --base-paths golf_mower_bringup \
  --packages-select golf_mower_bringup \
  --install-base golf_mower_bringup/install \
  --build-base golf_mower_bringup/build
source golf_mower_bringup/install/setup.bash
```

Before running launch files, source the built environments:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source unitree_lidar_ros2/install/setup.bash
source point_lio_unilidar-2.0.2/install/setup.bash
source patchwork-plusplus-ros/install/setup.bash
source elevation_mapping_cupy/install/setup.bash
source elevation_mapping_ros2/install/setup.bash
source golf_mower_description/install/setup.bash
source UM981/install/setup.bash
source Fields2Cover-main/install/setup.bash
source golf_mower_bringup/install/setup.bash
```

## Stage 5: Mapping and Localization/Navigation

Stage 5 has only two phases: first build and save the offline map, then load that map for localization and navigation. Run the source commands above in every new terminal.

### Phase 1: Mapping

Use fake RTK indoors when satellite positioning is unavailable:

```bash
ros2 launch golf_mower_bringup outdoor_segmented_mapping_stage5.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  patchwork_sensor_height:=0.75 \
  lidar_tf_z:=0.0 \
  imu_tf_z:=0.00667 \
  imu_quaternion_order:=wxyz \
  use_um981:=false \
  use_fake_rtk:=true \
  use_map_metadata_recorder:=true \
  fix_topic:=/fix \
  yaw_map_to_enu:=0.0 \
  use_pointlio_diagnostics:=true \
  launch_rviz:=true
```

Outdoors with a valid GNSS/RTK solution, replace these parameters in the same command:

```bash
use_um981:=true \
um981_port:=/dev/ttyUSB0 \
use_fake_rtk:=false
```

The UM981 gate accepts `rtk_fixed` GGA output by default, with at least 10 satellites and HDOP no greater than 1.5. Use `/um981/fix_quality` to inspect rejection reasons. Loosen `um981_require_rtk_fixed`, `um981_allow_rtk_float`, `um981_min_satellites`, or `um981_max_hdop` only for controlled diagnostics.

Each mapping launch creates a directory named with its start time, for example:

```text
golf_mower_bringup/maps/stage5_segmented/20260710_142530/
```

After mapping, verify that directory contains these files and stop the launch with `Ctrl+C`:

```text
ground_map.pcd
nonground_map.pcd
map_metadata.yaml
```

### Phase 2: Load Map and Localize/Navigate

Open a new terminal and run the source commands above again. Use fake RTK indoors:

```bash
ros2 launch golf_mower_bringup outdoor_segmented_nav_stage5.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  patchwork_sensor_height:=0.75 \
  lidar_tf_z:=0.0 \
  imu_tf_z:=0.00667 \
  imu_quaternion_order:=wxyz \
  use_um981:=false \
  use_fake_rtk:=true \
  use_rtk_map_localizer:=true \
  use_map_to_camera_init_adapter:=false \
  use_um981_heading:=false \
  rtk_map_to_camera_init_yaw:=0.0 \
  fix_topic:=/fix \
  use_coverage_planner:=true \
  coverage_area_file:=/home/ubuntu/unilidar_sdk2/golf_mower_bringup/config/coverage_test_area.yaml \
  use_coverage_geofence:=true \
  coverage_output_file:=~/.ros/golf_mower/coverage_path.yaml \
  coverage_dry_run:=true \
  launch_outdoor_rviz:=true
```

Outdoors with a valid GNSS/RTK solution, replace these parameters in the same command:

```bash
use_um981:=true \
um981_port:=/dev/ttyUSB0 \
use_fake_rtk:=false
```

Stage 5 uses CPU elevation mapping by default (`use_cuda_elevation:=false`). Enable CUDA only after confirming that the NVIDIA driver and GPU runtime are healthy. Map directories default to `~/unilidar_sdk2/golf_mower_bringup/maps/stage5_segmented`; set `GOLF_MOWER_MAP_ROOT` before launching to use another location.

#### Navigation Startup and Map Handling

Navigation automatically loads the newest timestamped directory under `stage5_segmented` that contains `ground_map.pcd`, `nonground_map.pcd`, and `map_metadata.yaml`. To load an older map, explicitly set `segmented_map_dir` and `map_metadata_path`.

Before navigation, the launch reads the `ground_map.pcd` header. If it contains zero points and `allow_offline_ground_recovery:=true`, it extracts a connected local low surface from `nonground_map.pcd`, writes `recovered_ground_map.pcd`, `recovered_nonground_map.pcd`, and `recovery_report.yaml`, then starts navigation. Failed quality checks, missing files, or invalid formats still abort the launch, and the original PCD files are never overwritten.

Fake RTK only validates offline map loading, the localization interface, and the TF chain; it does not represent real GNSS accuracy. For indoor navigation testing, place the robot near the mapping start pose with the same orientation, or adjust `rtk_map_to_camera_init_yaw`.

Check the map-localization chain:

```bash
ros2 topic hz /fix
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /map --once
ros2 topic echo /cmd_vel
```

#### `/cmd_vel` Output

`/cmd_vel` is the robot body velocity target produced by Nav2. Its message type is `geometry_msgs/msg/Twist`; it is not a motor speed, encoder measurement, or measured robot velocity. A differential-drive chassis mainly uses these fields:

| Field | Unit | Meaning |
| --- | --- | --- |
| `linear.x` | m/s | Forward/backward body velocity; positive is forward and negative is reverse. |
| `angular.z` | rad/s | Body yaw rate about the Z axis; positive turns left and negative turns right. |

`linear.y`, `linear.z`, `angular.x`, and `angular.y` should remain zero for a normal differential-drive chassis. An all-zero message is a stop target. For example:

```yaml
linear:
  x: 0.30
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.20
```

This command asks the robot to move forward at `0.30 m/s` while turning left at `0.20 rad/s`. In Stage 5, the safety controller forwards this stream to `/motor/cmd_vel`; `motor_driver_node.py` consumes that topic by default and uses the Serial V2 protocol in `motor/最新电机命令.md`. Its currently supported mapping is forward, reverse, forward arcs, in-place turns, and brake; it deliberately rejects reverse-turn commands because the documented `0x09` differential command does not define signed wheel directions.

#### Motor Driver

The motor driver is disabled by default. It sends a frame only when the requested motion or speed changes. A zero `/cmd_vel`, `/cmd_vel` timeout, disarm request, startup arming, and process shutdown send the configured stop command; the default is the documented `0x04` brake frame. `/motor_driver/status` reports commanded state only, not measured wheel motion.

First verify the frames without opening a serial port:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 launch golf_mower_bringup motor_driver.launch.py enabled:=true dry_run:=true
```

In another sourced terminal, publish a test target and inspect the status:

```bash
ros2 topic pub --once /motor/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.20}, angular: {z: 0.0}}'
ros2 topic echo /motor_driver/status
```

With the conservative default limits, this test logs `AA 02 0A 13 C9 55` (19% speed) followed by `AA 01 01 AC 55` (forward), then `AA 01 04 AF 55` after the timeout. Calibrate `min_speed_percent` and `max_speed_percent` only after a suspended-wheel test.

For a physical bench test, keep the drive wheels off the ground and confirm the controller's baud rate, `8N1`/flow-control settings, command interval, and emergency-stop circuit first. Start disarmed, then arm explicitly:

```bash
ros2 launch golf_mower_bringup motor_driver.launch.py \
  port:=<motor_serial_port> \
  baudrate:=<confirmed_baudrate> \
  dry_run:=false

ros2 service call /motor_driver/enable std_srvs/srv/SetBool '{data: true}'
```

Disarm and brake with:

```bash
ros2 service call /motor_driver/enable std_srvs/srv/SetBool '{data: false}'
```

#### Safety Controller

Stage 5 now uses this command path when both nodes are enabled:

```text
/cmd_vel -> /safety_controller -> /motor/cmd_vel -> /motor_driver
```

The safety controller starts disabled. It only forwards a fresh command after explicit enable, while `/pointlio/odom` is fresh. A `true` message on `/emergency_stop` immediately publishes zero velocity, latches the software E-stop, and requires an explicit reset after the input returns to `false`.

```bash
ros2 service call /motor_driver/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /safety_controller/enable std_srvs/srv/SetBool '{data: true}'

# Software E-stop test only; this is not a replacement for a wired E-stop.
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'
ros2 service call /safety_controller/reset_emergency_stop std_srvs/srv/Trigger '{}'
```

For Stage 5, add `use_motor_driver:=true use_safety_controller:=true motor_dry_run:=true` for a no-output integration test. Physical motor mode additionally requires `motor_port`, `motor_baudrate`, `motor_dry_run:=false`, `use_coverage_geofence:=true`, and `use_nav2:=true`; otherwise the launch stops with an error. Arm the motor driver first and the safety controller second. Do not use physical coverage execution until a wired emergency stop and wheel odometry are available.

### Stage 5 Parameter Reference

| Parameter | Example | Short meaning |
| --- | --- | --- |
| `initialize_type` | `1` | Lidar interface: `1` for USB serial and `2` for UDP. |
| `work_mode` | `8` | Unitree L2 serial operating mode. |
| `serial_port` | `/dev/ttyACM0` | Unitree L2 serial device, not the UM981 port. |
| `baudrate` | `4000000` | Unitree L2 serial baud rate. |
| `start_lidar_rotation` | `true` | Start lidar rotation with the launch. |
| `reset_lidar_after_set_mode` | `false` | Reset after setting lidar mode; use `false` on the current hardware. |
| `patchwork_sensor_height` | `0.75` | Measured lidar height above ground in meters. |
| `lidar_tf_z` | `0.0` | Lidar Z offset relative to `base_link`, in meters. |
| `imu_tf_z` | `0.00667` | Unitree internal IMU Z mounting offset, in meters. |
| `imu_quaternion_order` | `wxyz` | Unitree SDK IMU quaternion field order. |
| `use_um981` | indoor `false`, outdoor `true` | Start the real UM981 GNSS node. |
| `um981_port` | `/dev/ttyUSB0` | UM981 serial port, used only with `use_um981:=true`. |
| `um981_require_rtk_fixed` | `true` | Accept GGA data as `/fix` only when its quality is `rtk_fixed`. |
| `um981_allow_rtk_float` | `false` | Permit `rtk_float` when RTK fixed is unavailable; do not use for normal map initialization. |
| `um981_min_satellites` | `10` | Minimum satellites required before UM981 publishes a usable `/fix`. |
| `um981_max_hdop` | `1.5` | Maximum accepted GGA HDOP. |
| `use_fake_rtk` | indoor `true` | Generate simulated `/fix` from Point-LIO odometry for indoor testing. |
| `use_map_metadata_recorder` | mapping `true` | Write the localization datum and mapping start information to `map_metadata.yaml`. |
| `map_metadata_path` | default Stage 5 path | Metadata file written by mapping and read by navigation. |
| `fix_topic` | `/fix` | `NavSatFix` topic published by UM981 or fake RTK. |
| `yaw_map_to_enu` | `0.0` | Angle from ENU east to the map X axis, in radians. |
| `use_pointlio_diagnostics` | debugging `true` | Print cloud, IMU timing, and odometry jump diagnostics. |
| `launch_rviz` | `true` | Start RViz during mapping. |
| `use_rtk_map_localizer` | navigation `true` | Publish `map -> camera_init` from `/fix` and map metadata. |
| `allow_offline_ground_recovery` | `true` | Recover connected ground from nonground when ground is empty; abort navigation if recovery quality fails. |
| `use_map_to_camera_init_adapter` | `false` | Disable the duplicate static TF while the RTK localizer is active. |
| `use_um981_heading` | currently `false` | Use UM981 heading; disabled because valid INS heading is unavailable. |
| `rtk_map_to_camera_init_yaw` | `0.0` | Manual initial yaw in radians when UM981 heading is not used. |
| `launch_outdoor_rviz` | `true` | Start RViz during localization and navigation. |
| `use_coverage_planner` | default `false` | Start the Fields2Cover ROS2 node with Stage5 navigation. |
| `coverage_area_file` | example YAML path | Map-frame work boundary, exclusions, and robot planning parameters. |
| `use_coverage_geofence` | `true` for coverage missions | Apply the same `boundary/exclusions` to Nav2 local and global costmaps to prevent local avoidance from leaving the work area. |
| `coverage_output_file` | `~/.ros/golf_mower/coverage_path.yaml` | Destination for the generated path. |
| `coverage_mission_state_file` | `~/.ros/golf_mower/coverage_mission_state.yaml` | Stores segment states, retry counts, and the latest event for diagnostics; it never resumes motion automatically. |
| `coverage_dry_run` | currently `true` | Plan, publish, and save without submitting a Nav2 execution goal. |
| `coverage_path_pose_spacing` | `0.10` | Sampling distance for the published/saved path, in meters. |
| `coverage_nav_waypoint_spacing` | `0.75` | Sampling distance for Nav2 execution poses, in meters. |
| `use_motor_driver` | default `false` | Start the motor driver, which consumes `/motor/cmd_vel` by default. |
| `motor_port` | empty | Motor-controller serial port; empty is valid for dry-run tests. |
| `motor_baudrate` | `115200` | Provisional motor serial baud rate; confirm it with the controller documentation. |
| `motor_dry_run` | default `true` | Log serial frames only; `false` permits physical output after service arming. |
| `motor_cmd_vel_timeout_sec` | `0.50` | Maximum age of the motor velocity input before the driver sends the configured brake command. |
| `motor_max_linear_speed_mps` | `0.45` | `/cmd_vel.linear.x` magnitude mapped to the maximum configured motor percentage. |
| `motor_max_angular_speed_radps` | `0.70` | `/cmd_vel.angular.z` magnitude mapped to the maximum configured motor percentage. |
| `motor_min_speed_percent` | `10` | Conservative percentage used by the smallest nonzero command. |
| `motor_max_speed_percent` | `30` | Conservative maximum motor percentage before real-wheel calibration. |
| `use_safety_controller` | default `false` | Start the software gate between Nav2 `/cmd_vel` and `/motor/cmd_vel`. Required for physical motor output. |
| `safety_emergency_stop_topic` | `/emergency_stop` | `std_msgs/Bool`; `true` latches a software stop and publishes zero velocity. |
| `safety_cmd_vel_timeout_sec` | `0.50` | Maximum age of the Nav2 velocity target before the safety controller blocks output. |
| `safety_odom_timeout_sec` | `0.50` | Maximum age of `/pointlio/odom` before the safety controller blocks output. |

For Boolean parameters, `true` enables a function and `false` disables it. Angles use radians; positions and heights use meters.

## Coverage Path Planning

The planner reads `golf_mower_bringup/config/coverage_test_area.yaml` and generates mowing swaths and turns in the `map` frame.

Start the planner in terminal A and keep it running:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source Fields2Cover-main/install/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 launch golf_mower_bringup coverage_planner.launch.py \
  area_file:=/home/ubuntu/unilidar_sdk2/golf_mower_bringup/config/coverage_test_area.yaml \
  output_file:=~/.ros/golf_mower/coverage_path.yaml \
  dry_run:=true \
  launch_rviz:=true
```

The node plans automatically at startup and displays the result in RViz; initial display does not require `replan`.

Main interfaces:

| Interface | Type | Purpose |
| --- | --- | --- |
| `/coverage_path` | `nav_msgs/msg/Path` | Continuous coverage path in the `map` frame. |
| `/coverage_markers` | `visualization_msgs/msg/MarkerArray` | RViz work boundary, exclusions, and coverage path. |
| `/coverage_planner/status` | `std_msgs/msg/String` | Current segment and completed, blocked, and canceled counts. |
| `/coverage_planner/replan` | `std_srvs/srv/Trigger` | Reload the YAML file and replan. |
| `/coverage_planner/execute` | `std_srvs/srv/Trigger` | Start segmented Nav2 execution; rejected while `dry_run=true`. |
| `/coverage_planner/cancel` | `std_srvs/srv/Trigger` | Cancel the active goal and remaining coverage mission. |

After editing the YAML file, replan from terminal B:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source Fields2Cover-main/install/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 service call /coverage_planner/replan std_srvs/srv/Trigger '{}'
```

`'{}'` is the empty request required by `std_srvs/srv/Trigger`; keep it unchanged.

The Stage 5 phase-2 command above already enables a safe coverage dry run. Tune `coverage_mission_state_file`, `coverage_segment_max_waypoints`, `coverage_segment_max_retries`, or `coverage_segment_timeout_sec` only when needed. `coverage_segment_max_waypoints` is advisory and never splits a complete mowing swath or turn; a final failure stops by default (`coverage_continue_after_blocked:=false`).

Core YAML parameters:

| YAML parameter | Unit | Meaning |
| --- | --- | --- |
| `robot.width` | m | Physical chassis width used by Fields2Cover to reserve headland space. |
| `robot.coverage_width` | m | Effective cutting width per pass; determines adjacent swath spacing. |
| `robot.min_turning_radius` | m | Minimum permitted radius of the robot center trajectory. |
| `robot.cruise_speed` | m/s | Desired speed on straight mowing swaths; currently Fields2Cover path metadata only. |
| `robot.turn_speed` | m/s | Desired speed on connections and turns; currently Fields2Cover path metadata only. |
| `planner.headland_swaths` | passes | Number of boundary headland passes reserved for turns. |
| `planner.swath_angle_deg` | degree | Swath direction; use `null` to let Fields2Cover choose it. |

`planner.headland_swaths` uses the robot width as its unit. It specifies the width reserved inside the boundary for turning; it is not the number of laps the robot must drive:

```text
┌──────────────────────────────┐  Work-area boundary
│      Headland turn area      │
│    ┌────────────────────┐    │
│    │  → → → → → → → →  │    │
│    │  ← ← ← ← ← ← ← ←  │    │  Mowing swaths
│    │  → → → → → → → →  │    │
│    └────────────────────┘    │
│      Headland turn area      │
└──────────────────────────────┘
```

The approximate reserved width is `robot.width × planner.headland_swaths`. For example, `robot.width: 0.5` and `headland_swaths: 10` reserve about `5 m` inside the boundary for turns and connections between mowing swaths. Too small a value can make turns cross the boundary; too large a value reduces the usable central mowing area.

`boundary` and `exclusions` use meters in the offline map's `map` frame. `cruise_speed` and `turn_speed` do not currently change Nav2 speed. Keep `dry_run:=true` until the chassis and safety system are complete.

### Generate a Candidate Boundary from Point Cloud

After mapping, generate a convex candidate boundary from the largest connected ground region. Replace `<map-directory>` with the actual timestamped directory:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 run golf_mower_bringup coverage_boundary_from_pcd.py \
  --ground-pcd <map-directory>/ground_map.pcd \
  --output <map-directory>/coverage_candidate.yaml \
  --resolution 0.25 \
  --connect-gap-cells 2 \
  --robot-width 1.50 \
  --coverage-width 1.00 \
  --min-turning-radius 1.0 \
  --headland-swaths 3
```

`resolution` is the ground raster cell size. `connect-gap-cells` bridges small scanning gaps. The resulting `coverage_candidate.yaml` can be passed directly as `coverage_area_file` for a `dry_run` visualization. It is a **convex-hull candidate** of the largest connected ground region and may include concavities, unobserved holes, roads, or areas where mowing is prohibited. Verify it in RViz and add `exclusions` manually before commanding robot motion.

An empty `ground_map.pcd` cannot produce a candidate. First run Stage 5 phase 2 ground recovery, then use `recovered_ground_map.pcd` from the same directory as the input.

### Stage 5 Coverage Execution

Before starting Stage 5 phase 2, prepare a complete timestamped map directory and a reviewed `coverage_test_area.yaml`. Its `boundary` and `exclusions` use the offline map's `map` coordinates; enter the actual chassis width, cutting width, and minimum turning radius. For indoor tests, return near the mapping start pose with the same heading and use fake RTK.

The phase-2 data flow is:

```text
offline PCD -> Nav2 /map -> RTK or fake RTK map initialization
            -> Point-LIO live pose -> Fields2Cover path -> Nav2 waypoints -> /cmd_vel
```

Start with `coverage_dry_run:=true`. The planner publishes `/coverage_path` and `/coverage_markers`, saves `coverage_output_file`, and does not move the robot. After checking localization and the path, restart with `coverage_dry_run:=false` and explicitly start the mission:

```bash
ros2 service call /coverage_planner/execute std_srvs/srv/Trigger '{}'
```

The task manager submits complete swath-and-turn sequences to Nav2, records `PENDING`, `ACTIVE`, `COMPLETED`, `BLOCKED`, and `CANCELED`, and retries or times out failed segments according to the configured limits. Inspect or cancel a mission with:

```bash
ros2 topic echo /coverage_planner/status
ros2 service call /coverage_planner/cancel std_srvs/srv/Trigger '{}'
```

Fields2Cover handles fixed obstacles defined in `exclusions`; Nav2 handles temporary obstacles through its local costmap. A final blocked segment stops by default (`coverage_continue_after_blocked:=false`). The system does not yet create a recovery plan for areas missed while avoiding obstacles, so keep physical output disabled outside controlled low-speed tests.

## Earlier Debug Stages

The earlier stages are useful for isolating one layer at a time. They are not the preferred full-system entry point once Stage 5 is working.

### Robot Description

```bash
ros2 launch golf_mower_description description.launch.py launch_rviz:=true
```

### Indoor Point-LIO Test

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

Check:

```bash
ros2 topic hz /unilidar/cloud
ros2 topic hz /unilidar/imu
ros2 topic hz /pointlio/odom
```

### Stage 1: Outdoor Elevation Mapping

```bash
ros2 launch golf_mower_bringup outdoor_elevation_stage1.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  launch_outdoor_rviz:=true
```

Check:

```bash
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /elevation_mapping_node/elevation_map_filter
ros2 run golf_mower_bringup grid_map_inspect.py
```

### Stage 2: Traversability Occupancy Grid

```bash
ros2 launch golf_mower_bringup outdoor_elevation_stage2.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  launch_outdoor_rviz:=true
```

Check:

```bash
ros2 topic hz /elevation/traversability_grid
ros2 topic echo /elevation/traversability_grid --once
```

### Stage 3: Nav2 With Elevation Grid

```bash
ros2 launch golf_mower_bringup outdoor_nav2_stage3.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  launch_outdoor_rviz:=true
```

### Stage 4: Patchwork++ Segmentation

```bash
ros2 launch golf_mower_bringup outdoor_patchwork_stage4.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  patchwork_sensor_height:=0.75 \
  launch_outdoor_rviz:=true
```

Check:

```bash
ros2 topic hz /ground_segmentation/ground
ros2 topic hz /ground_segmentation/nonground
```

## Common Parameter Reference

### Lidar Driver Parameters

| Parameter | Typical value | Meaning |
| --- | --- | --- |
| `initialize_type` | `1` serial, `2` UDP | Unitree L2 initialization mode. Use `1` for USB serial. |
| `work_mode` | `8` serial, `0` UDP | Unitree L2 work mode. Serial L2 testing commonly uses `8`. |
| `serial_port` | `/dev/ttyACM0` | Serial device path for Unitree L2. |
| `baudrate` | `4000000` | Serial baudrate for Unitree L2. |
| `start_lidar_rotation` | `true` | Start lidar motor rotation from launch. |
| `reset_lidar_after_set_mode` | `false` | Reset lidar after mode configuration. Set `false` if reset causes unstable startup. |
| `use_system_timestamp` | `false` | Use host time instead of sensor timestamps. Prefer `false` when sensor timestamps are valid. |

### Point-LIO and IMU Parameters

| Parameter | Typical value | Meaning |
| --- | --- | --- |
| `pointlio_config_file` | `unilidar_l2_ros2_no_pcd.yaml` | Point-LIO configuration file. |
| `imu_quaternion_order` | `wxyz` | Quaternion field order expected from the Unitree IMU adapter. |
| `imu_angular_velocity_scale` | `0.017453292519943295` | Converts degrees/s to rad/s. Use `1.0` only if the driver already publishes rad/s. |
| `imu_linear_acceleration_scale` | `1.0` | Scale factor for linear acceleration. |
| `use_static_pointlio_pose` | `false` | Use a fixed pose for testing instead of live Point-LIO output. |

### TF and Sensor Mounting Parameters

| Parameter | Meaning |
| --- | --- |
| `lidar_tf_x/y/z` | Lidar position relative to `base_link`, in meters. |
| `lidar_tf_roll/pitch/yaw` | Lidar orientation relative to `base_link`, in radians. |
| `imu_tf_x/y/z` | IMU position relative to `base_link`, in meters. |
| `imu_tf_roll/pitch/yaw` | IMU orientation relative to `base_link`, in radians. |
| `map_to_camera_init_x/y/z` | Manual static offset from `map` to Point-LIO `camera_init`. |
| `map_to_camera_init_roll/pitch/yaw` | Manual static rotation from `map` to `camera_init`, in radians. |
| `use_map_to_camera_init_adapter` | Publish the static `map -> camera_init` adapter. Set `false` when `use_rtk_map_localizer:=true`. |

### Patchwork++ Parameters

| Parameter | Typical value | Meaning |
| --- | --- | --- |
| `patchwork_cloud_topic` | `/unilidar/cloud` | Input cloud for ground segmentation. |
| `patchwork_sensor_height` | `0.75` | Measured lidar mounting height above ground. Do not leave this at `0` for outdoor use. |
| `patchwork_min_r` | `0.2` | Minimum radial range used by Patchwork++. |
| `patchwork_max_r` | `40.0` | Maximum radial range used by Patchwork++. |
| `patchwork_log_every_n` | `60` | Patchwork++ log throttle interval in frames. |
| `min_ground_points` | `100` | Minimum points for accepting live Patchwork++ ground. Short failures hold the previous elevation map; persistent failures recover only the local low surface instead of treating the full raw cloud as ground. |

### UM981 ROS Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `/fix` | `sensor_msgs/NavSatFix` | GNSS/RTK position from GGA, INSPVAX, or DRPVA. |
| `/um981/fix_quality` | `std_msgs/String` | Parsed UM981 quality, satellite, HDOP, and acceptance decision. |
| `/imu/data_raw` | `sensor_msgs/Imu` | Not used in the current robot flow. The tested UM981 USB output did not emit RAWIMUX. |
| `/um981/heading` | `std_msgs/Float64` | Not used in the current robot flow. Reserved for valid INS heading output. |
| `/um981/ins_attitude` | `geometry_msgs/Vector3Stamped` | Not used in the current robot flow. |

## Debugging Notes

Keep the robot and lidar still for 5 to 10 seconds during Point-LIO initialization. Moving the robot, touching the lidar, or pulling cables during startup can corrupt the initial IMU bias and gravity estimate, causing map drift.

Common symptoms:

- `Patchwork++ produced empty ground cloud`: check `patchwork_sensor_height` and point cloud axis convention.
- `Lookup would require extrapolation into the future`: TF is slightly behind the point cloud timestamp. This can affect segmented map accumulation but does not always mean Point-LIO is drifting.
- `/ground_segmentation/ground` stays empty: do not keep `patchwork_sensor_height:=0`; use the real lidar height.
- `/fix` has `STATUS_NO_FIX`: the UM981 is indoors or has no valid satellite solution. Use fake RTK indoors, or test GNSS outdoors.

## Next Development Tasks

1. Connect a wired, normally-closed emergency-stop circuit that removes motor power independently of ROS; connect its state to `/emergency_stop` for diagnostics.
2. Confirm the motor controller serial settings, ACK/NACK behavior, command watchdog, status frame, and signed differential-wheel encoding; then add feedback parsing and configurable command keepalive.
3. Measure the complete chassis envelope, then keep Fields2Cover width, Nav2 footprint, inflation radius, and exclusion margins in one calibrated geometry source.
4. Add wheel encoders, calibrate wheel radius/wheelbase, publish `/wheel/odom`, and fuse wheel odometry, RTK, IMU, and Point-LIO with `robot_localization`.
5. Validate the UM981 quality thresholds outdoors against the receiver's actual GGA output; request and validate `INSPVAX` or `DRPVA` before enabling heading-based initialization.
6. Add Nav2 collision monitoring, speed zones for mowing swaths, turns, obstacle proximity, and degraded localization; keep `coverage_continue_after_blocked:=false` for real vehicles.
7. Improve offline map semantics with outlier filtering, bounded map extents, ground-density checks, and a reviewed free-space reconstruction method.
8. Track actual cutter state and measured trajectory, derive uncovered regions after obstacle avoidance, and generate recovery coverage missions.
9. Add pseudo-terminal motor tests, launch tests, recorded-bag regression tests, and a hardware-in-the-loop checklist before field operation.
