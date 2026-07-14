# Golf Mower Robot ROS2 Workspace

English | [中文](README_CN.md)

This workspace contains the ROS2 algorithm stack for a golf-course mowing robot. It is built around the Unitree L2 lidar, Point-LIO, Patchwork++ ground segmentation, elevation/traversability mapping, Nav2 navigation, and UM981 RTK/INS localization.

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
| Chassis motor driver | Not implemented | No node currently consumes `/cmd_vel` to drive the physical wheels. |
| Wheel encoder odometry | Not implemented | No `/wheel/odom` source or calibrated chassis kinematics. |
| Mower actuator | Model only | `mower_tool` exists in URDF, but there is no blade motor interface, feedback, or fault handling. |
| Coverage path planning | Integrated with ROS2 | Reads map-frame boundaries and exclusions from YAML, publishes paths/markers, saves results, and provides an optional Nav2 action; dry-run is the default. |
| Mission manager | Not implemented | No mowing task state machine, pause/resume, return, or coverage progress tracking. |
| Safety system | Not implemented | No integrated emergency-stop state, command watchdog, geofence, blade interlock, or safety controller. |

The repository currently provides a perception, mapping, localization-initialization, and navigation-planning prototype. It is not yet a complete autonomous mower: physical motion, blade actuation, coverage execution, and safety interlocks must be implemented and validated before field mowing.

The next system milestone is a safe chassis control loop: `/cmd_vel` to motor commands, wheel encoder odometry, a hardware emergency stop, and a command-timeout stop. After that, complete continuous state estimation, physical coverage-path execution, the mower actuator, and the mission/safety state machine.

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
| `golf_mower_bringup` | Main launch files, configuration, diagnostics, map utilities, and Nav2 integration. |
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

This command asks the robot to move forward at `0.30 m/s` while turning left at `0.20 rad/s`. The physical chassis driver must subscribe to `/cmd_vel`, convert it into left and right wheel targets using the chassis kinematics, and send them through the motor-controller protocol. It must also enforce velocity limits, command-timeout stops, communication-failure stops, and a hardware emergency stop. That physical chassis driver is not implemented in this repository yet, so observing `/cmd_vel` does not mean the wheels will move.

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
| `coverage_output_file` | `~/.ros/golf_mower/coverage_path.yaml` | Destination for the generated path. |
| `coverage_dry_run` | currently `true` | Plan, publish, and save without submitting a Nav2 execution goal. |
| `coverage_path_pose_spacing` | `0.10` | Sampling distance for the published/saved path, in meters. |
| `coverage_nav_waypoint_spacing` | `0.75` | Sampling distance for Nav2 execution poses, in meters. |

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
| `/coverage_planner/replan` | `std_srvs/srv/Trigger` | Reload the YAML file and replan. |
| `/coverage_planner/execute` | `std_srvs/srv/Trigger` | Submit poses to Nav2; rejected while `dry_run=true`. |

After editing the YAML file, replan from terminal B:

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source Fields2Cover-main/install/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 service call /coverage_planner/replan std_srvs/srv/Trigger '{}'
```

`'{}'` is the empty request required by `std_srvs/srv/Trigger`; keep it unchanged.

For Stage5 phase 2, append:

```bash
use_coverage_planner:=true \
coverage_area_file:=/home/ubuntu/unilidar_sdk2/golf_mower_bringup/config/coverage_test_area.yaml \
coverage_dry_run:=true
```

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

### Complete Stage 5 Coverage Workflow

Prepare the following before starting phase 2:

1. A complete timestamped map directory from phase 1 containing `ground_map.pcd`, `nonground_map.pcd`, and `map_metadata.yaml`. The latest complete directory is selected automatically; use `segmented_map_dir` and `map_metadata_path` to select a specific run.
2. `coverage_test_area.yaml`. Its `boundary` and `exclusions` must use the offline map's `map` coordinates. The point-cloud map does not automatically define the mowing boundary. Also enter the real chassis width, cutting width, and minimum turning radius.
3. For indoor testing, return the robot close to the mapping start pose with the same heading and use fake RTK. For outdoor operation, prepare a valid UM981 solution and the correct map-heading alignment.

Phase 2 runs this pipeline:

```text
Load ground/nonground PCD files
  -> build the Nav2 /map
  -> RTK or fake RTK establishes map -> camera_init
  -> Point-LIO supplies the live local pose
  -> Fields2Cover reads coverage_test_area.yaml
  -> publish the coverage path
  -> Nav2 follows the coverage waypoints
  -> output /cmd_vel to the chassis driver
```

Keep `coverage_dry_run:=true` during initial integration. The node plans automatically and RViz displays the boundary, exclusions, and path, but no Nav2 goal is executed. After verifying localization and the path, restart phase 2 with `coverage_dry_run:=false`, then submit the path from another sourced terminal:

```bash
ros2 service call /coverage_planner/execute std_srvs/srv/Trigger '{}'
```

Main outputs:

| Output | Purpose |
| --- | --- |
| `/map` | Nav2 occupancy grid built from the offline ground/non-ground clouds. |
| `/coverage_path` | Complete continuous coverage path in the `map` frame. |
| `/coverage_markers` | Boundary, exclusions, and path visualization in RViz. |
| `~/.ros/golf_mower/coverage_path.yaml` | Saved path file; change it with `coverage_output_file`. |
| Nav2 `NavigateThroughPoses` goal | Navigation waypoints sampled from the coverage path. |
| `/cmd_vel` | Body velocity target produced by Nav2 for the chassis driver. |

The real chassis driver is not implemented in this repository yet. Offline-map loading, localization, coverage planning, and Nav2 velocity-output tests are available, but `/cmd_vel` alone does not drive the physical wheels.

### Obstacle Handling and Remaining Work

Fields2Cover decides where mowing is required, while Nav2 tracks the path safely. They do not directly conflict, but the current system cannot automatically recover areas missed during obstacle avoidance:

| Obstacle type | Handling |
| --- | --- |
| Fixed trees, buildings, bunkers, and ponds | Add them to `coverage_test_area.yaml` under `exclusions` so Fields2Cover plans around them. |
| People, vehicles, and temporary equipment | Let the Nav2 local costmap slow, stop, or locally avoid them. |
| Long-term blockage | The current Nav2 task may fail; a future task manager must skip the segment and schedule it for recovery. |

Complete the following before autonomous mowing:

1. Split the full coverage path into independently executable swaths or path segments.
2. Add a coverage task manager with `PENDING`, `ACTIVE`, `COMPLETED`, `BLOCKED`, and `RETRY` states.
3. Wait or use local avoidance for short blockages; after a timeout, cancel the segment, mark it incomplete, and continue with another segment.
4. Maintain a mowed-area grid from the robot's measured trajectory and cutter width instead of assuming that every planned path was completed.
5. After the task or when an obstacle clears, derive recovery areas from the uncovered grid and invoke Fields2Cover again.
6. Stop safely on boundary violations, localization loss, costmap failure, communication timeout, or emergency-stop activation.

Keep `coverage_dry_run:=true` until the task-management and safety mechanisms are implemented, or restrict low-speed Nav2 tracking tests to a controlled area.

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

## Parameter Reference

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

## Development Roadmap

- Enable and calibrate UM981 INS heading only after valid heading output is available on the tested serial port.
- Calibrate the RTK antenna lever arm.
- Fuse RTK, wheel odometry, IMU, and Point-LIO with `robot_localization`.
- Calibrate coverage parameters against the real course boundary, exclusions, and mowing width.
- Validate physical Nav2 coverage execution after the safe chassis loop is complete.
- Add mower actuator state, emergency stop, boundary protection, and no-go zones.
