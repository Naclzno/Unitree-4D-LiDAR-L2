# Golf Mower Robot ROS2 Workspace

这是一个面向高尔夫球场除草机器人的 ROS2 算法工作区。项目围绕 Unitree L2 激光雷达，集成了激光雷达驱动、Point-LIO 激光惯导里程计、地面分割、高程/可通行性建图、Nav2 导航和覆盖路径规划相关算法。

## 功能概览

当前系统主链路：

```text
Unitree L2
  -> /unilidar/cloud, /unilidar/imu
  -> Point-LIO
  -> /pointlio/odom, /pointlio/cloud_registered, /pointlio/laser_map
  -> Patchwork++ ground segmentation
  -> elevation mapping / traversability grid
  -> Nav2 navigation
```

主要能力：

- Unitree L2 串口或 UDP 采集点云和 IMU。
- Point-LIO 输出实时里程计和点云地图。
- Patchwork++ 分割地面点和非地面障碍点。
- elevation mapping 生成高程图和可通行性图。
- GridMap 转 OccupancyGrid，供 Nav2 costmap 使用。
- Nav2 室内桌面联调、室外高程导航、分割地图导航。
- Fields2Cover 预留用于高尔夫球场全覆盖割草路径规划。

## 目录说明

| 目录 | 作用 |
| --- | --- |
| `unitree_lidar_sdk` | Unitree L2 原始 C++ SDK。 |
| `unitree_lidar_ros2` | Unitree L2 ROS2 驱动，发布 `/unilidar/cloud` 和 `/unilidar/imu`。 |
| `point_lio_unilidar-2.0.2` | Unitree L2 适配版 Point-LIO。 |
| `patchwork-plusplus-ros` | ROS2 Patchwork++ 地面分割。 |
| `elevation_mapping_cupy` | GPU/CuPy 高程建图。 |
| `elevation_mapping_ros2` | CPU-only 高程建图兼容实现。 |
| `golf_mower_description` | 机器人 URDF/Xacro 和固定传感器 frame。 |
| `golf_mower_bringup` | 本项目主要 launch、配置、诊断脚本和 Nav2 插件。 |
| `Fields2Cover-main` | 农业/割草全覆盖路径规划库。 |
| `robot_localization-rolling-devel` | 后续 RTK、轮速、IMU、里程计融合预留。 |
| `autoware` | 自动驾驶栈源码，当前不是主链路必需模块。 |

## 环境准备

已按 ROS2 Humble 组织使用。每个终端先 source 基础环境：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
```

如果使用串口雷达，确认设备存在：

```bash
ls /dev/ttyACM*
```

常用串口参数：

- `initialize_type:=1`
- `work_mode:=8`
- `serial_port:=/dev/ttyACM0`
- `baudrate:=4000000`

常用 UDP 参数：

- `initialize_type:=2`
- `work_mode:=0`
- 本机 IP 通常配置为 `192.168.1.2`

## 构建步骤

建议按依赖顺序分别构建。构建前先进入工作区根目录：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
```

构建 Unitree ROS2 驱动：

```bash
colcon --log-base unitree_lidar_ros2/log build \
  --base-paths unitree_lidar_ros2/src \
  --install-base unitree_lidar_ros2/install \
  --build-base unitree_lidar_ros2/build
source unitree_lidar_ros2/install/setup.bash
```

构建 Point-LIO：

```bash
colcon --log-base point_lio_unilidar-2.0.2/log build \
  --base-paths point_lio_unilidar-2.0.2 \
  --install-base point_lio_unilidar-2.0.2/install \
  --build-base point_lio_unilidar-2.0.2/build
source point_lio_unilidar-2.0.2/install/setup.bash
```

构建 Patchwork++：

```bash
colcon --log-base patchwork-plusplus-ros/log build \
  --base-paths patchwork-plusplus-ros \
  --install-base patchwork-plusplus-ros/install \
  --build-base patchwork-plusplus-ros/build
source patchwork-plusplus-ros/install/setup.bash
```

构建高程建图模块：

```bash
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
```

构建机器人描述和 bringup：

```bash
colcon --log-base golf_mower_description/log build \
  --base-paths golf_mower_description \
  --install-base golf_mower_description/install \
  --build-base golf_mower_description/build
source golf_mower_description/install/setup.bash

colcon --log-base golf_mower_bringup/log build \
  --packages-select golf_mower_bringup \
  --install-base golf_mower_bringup/install \
  --build-base golf_mower_bringup/build
source golf_mower_bringup/install/setup.bash
```

## 使用步骤

每次运行前建议 source 所需工作区：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source unitree_lidar_ros2/install/setup.bash
source point_lio_unilidar-2.0.2/install/setup.bash
source patchwork-plusplus-ros/install/setup.bash
source elevation_mapping_cupy/install/setup.bash
source elevation_mapping_ros2/install/setup.bash
source golf_mower_description/install/setup.bash
source golf_mower_bringup/install/setup.bash
```

### 1. 查看机器人模型

```bash
ros2 launch golf_mower_description description.launch.py launch_rviz:=true
```

### 2. 室内 Point-LIO 测试

串口 L2：

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

检查输出：

```bash
ros2 topic hz /unilidar/cloud
ros2 topic hz /unilidar/imu
ros2 topic hz /pointlio/odom
```

### 3. 室外高程建图 Stage 1

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

检查高程图：

```bash
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /elevation_mapping_node/elevation_map_filter
ros2 run golf_mower_bringup grid_map_inspect.py
```

### 4. 可通行性栅格 Stage 2

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

检查 Nav2 可用的栅格：

```bash
ros2 topic hz /elevation/traversability_grid
ros2 topic echo /elevation/traversability_grid --once
```

### 5. Patchwork++ 地面分割 Stage 4

雷达安装高度要填写实际值，不建议长期使用 `0`：

```bash
ros2 launch golf_mower_bringup outdoor_patchwork_stage4.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  patchwork_sensor_height:=0.80 \
  launch_outdoor_rviz:=true
```

检查地面分割：

```bash
ros2 topic hz /ground_segmentation/ground
ros2 topic hz /ground_segmentation/nonground
```

### 6. 分割建图 Stage 5

用于生成 `ground_map.pcd` 和 `nonground_map.pcd`：

```bash
ros2 launch golf_mower_bringup outdoor_segmented_mapping_stage5.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  patchwork_sensor_height:=0.80 \
  use_pointlio_diagnostics:=true
```

输出目录：

```text
golf_mower_bringup/maps/stage5_segmented/
```

### 7. 分割地图导航 Stage 5

```bash
ros2 launch golf_mower_bringup outdoor_segmented_nav_stage5.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  patchwork_sensor_height:=0.80 \
  launch_outdoor_rviz:=true
```

Nav2 输出速度：

```bash
ros2 topic echo /cmd_vel
```

## 调试建议

Point-LIO 初始化时，雷达和车体应保持静止 5 到 10 秒。启动瞬间移动、碰撞雷达或线缆拉扯都可能导致初始 IMU bias 和重力估计错误，表现为地图漂移。

启用 Point-LIO 输入诊断：

```bash
ros2 launch golf_mower_bringup outdoor_segmented_mapping_stage5.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  use_pointlio_diagnostics:=true
```

重点看：

- `cloud time[min,max,span]` 是否约为单帧扫描周期。
- `imu_delta` / `cloud_delta` 是否在合理范围内。
- 静止时 `acc_norm_mean` 是否接近 `9.81`。
- 转动雷达时 `gyro_norm_mean` 是否符合实际角速度。如果明显小 57 倍，尝试 `imu_angular_velocity_scale:=1.0`。

常见问题：

- `Patchwork++ produced empty ground cloud`：检查 `patchwork_sensor_height` 和点云坐标轴方向。
- `Lookup would require extrapolation into the future`：TF 发布时间略慢于点云时间，通常影响分割地图累计，不一定影响 Point-LIO 本身。
- `/ground_segmentation/ground` 一直为空：不要把 `patchwork_sensor_height` 长期设为 `0`，应使用雷达实际离地高度。

## 后续开发方向

- 接入 RTK、轮速和 IMU，通过 `robot_localization` 融合为稳定 `map -> odom -> base_link`。
- 使用 Fields2Cover 根据球场边界和割草宽度生成覆盖路径。
- 将覆盖路径转换为 Nav2 waypoint/action。
- 将割草机构状态、急停、边界保护和禁入区加入任务管理。
