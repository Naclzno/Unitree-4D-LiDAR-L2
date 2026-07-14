# 高尔夫除草机器人 ROS2 工作区

[English](README.md) | 中文

这是一个面向高尔夫球场除草机器人的 ROS2 算法工作区。当前系统围绕 Unitree L2 激光雷达、Point-LIO、Patchwork++ 地面分割、高程/可通行性建图、Nav2 导航，以及 UM981 RTK/INS 定位构建。

当前完整室外流程是 Stage 5：

```text
Unitree L2
  -> /unilidar/cloud, /unilidar/imu
  -> Point-LIO
  -> /pointlio/odom, /pointlio/cloud_registered, /pointlio/laser_map
  -> Patchwork++ 地面/非地面分割
  -> 离线分割 PCD 地图
  -> RTK/INS 在离线地图中初始化机器人位置和朝向
  -> Nav2 加载离线地图导航
```

## 功能概览

- 通过串口或 UDP 读取 Unitree L2 点云和 IMU。
- 使用 Point-LIO 输出激光惯导里程计和局部点云地图。
- 使用 Patchwork++ 分割地面点和障碍物点。
- 保存离线分割地图：`ground_map.pcd` 和 `nonground_map.pcd`。
- 保存离线地图的 RTK 地理参考元数据。
- 导航时加载离线地图。
- 使用 UM981 `/fix` 初始化机器人在离线地图中的位置。
- 当前 yaw 手动给定，UM981 INS heading 暂不用于机器人流程。
- 室内没有 RTK/GNSS 信号时，可用 fake RTK 测试完整链路。
- 使用 Fields2Cover 从 `map` 坐标边界和禁入区生成覆盖路径，并发布到 ROS2。
- 保留 Stage 1/2/3/4 用于逐层调试。

## 当前项目完成度

| 子系统 | 状态 | 当前范围 |
| --- | --- | --- |
| Unitree L2点云和IMU | 已接入 | 串口/UDP驱动发布 `/unilidar/cloud` 和 `/unilidar/imu`。 |
| Point-LIO里程计 | 已接入 | 发布激光惯导里程计和配准点云。 |
| Patchwork++地面分割 | 已接入，仍需标定 | ground/nonground话题已连接；ground仍可能为空，需要继续验证安装外参和坐标轴。 |
| 实时高程和可通行性 | 已接入 | 支持GPU或CPU高程建图，并带保守的局部最低表面fallback。 |
| 离线分割地图 | 已接入 | 按时间戳保存ground、nonground和地理元数据，支持离线ground恢复。 |
| UM981 GNSS位置 | 部分接入 | 使用GGA `/fix` 初始化离线地图位置；UM981 IMU/INS heading尚未使用。 |
| 连续定位融合 | 未接入 | RTK目前只初始化 `map -> camera_init`；轮速、RTK、IMU和Point-LIO尚未持续融合。 |
| Nav2规划和避障 | 算法层已接入 | 使用离线全局地图和实时局部地形/障碍生成 `/cmd_vel`。 |
| 底盘电机驱动 | 未实现 | 当前没有节点消费 `/cmd_vel` 并驱动物理车轮。 |
| 轮速里程计 | 未实现 | 没有 `/wheel/odom` 数据源和经过标定的底盘运动学。 |
| 割草执行器 | 仅模型 | URDF包含 `mower_tool`，但没有刀盘电机接口、反馈和故障处理。 |
| 覆盖路径规划 | 已接入ROS2 | 从YAML读取 `map` 边界和禁入区，发布路径/Marker、保存结果，并提供可选Nav2 action；默认dry-run。 |
| 任务管理 | 未实现 | 没有割草任务状态机、暂停/恢复、返航和覆盖进度管理。 |
| 安全系统 | 未实现 | 没有集成急停状态、速度命令看门狗、电子围栏、刀盘互锁和安全控制器。 |

当前仓库属于感知、建图、定位初始化和导航规划原型，还不是完整的自主除草机器人。实体运动、刀盘执行、覆盖作业和安全互锁完成并验证前，不能用于真实球场自动割草。

下一系统里程碑应先完成安全底盘闭环：`/cmd_vel` 转电机命令、轮速里程计、硬件急停和命令超时停车。之后再依次完成持续状态融合、覆盖路径实车执行、割草执行器以及任务/安全状态机。

## 目录说明

| 目录 | 作用 |
| --- | --- |
| `unitree_lidar_sdk` | Unitree L2 原始 C++ SDK。 |
| `unitree_lidar_ros2` | Unitree L2 ROS2 驱动，发布 `/unilidar/cloud` 和 `/unilidar/imu`。 |
| `point_lio_unilidar-2.0.2` | 适配 Unitree L2 的 Point-LIO。 |
| `patchwork-plusplus-ros` | ROS2 Patchwork++ 地面分割。 |
| `elevation_mapping_cupy` | GPU/CuPy 高程建图。 |
| `elevation_mapping_ros2` | CPU 高程建图兼容包。 |
| `golf_mower_description` | 机器人 URDF/Xacro 和传感器 frame。 |
| `golf_mower_bringup` | 主要 launch、配置、诊断、地图工具和 Nav2 集成。 |
| `UM981` | UM981 GNSS/RTK/INS Python SDK 和 ROS2 节点。 |
| `Fields2Cover-main` | Fields2Cover覆盖路径规划库，由 `golf_mower_bringup` 的ROS2节点调用。 |
| `robot_localization-rolling-devel` | 后续 RTK、轮速、IMU、激光里程计融合预留。 |
| `autoware` | Autoware 源码，当前主链路不依赖。 |

## 环境准备

| 环境组件 | 版本或型号 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS（Jammy Jellyfish），x86_64 |
| 显卡 | NVIDIA GeForce RTX 3060 Lite Hash Rate |
| NVIDIA驱动 | `535.309.01` |
| CUDA | CUDA 12.2 |
| ROS 2 | ROS 2 Humble Hawksbill |

每个终端先执行：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
```

检查 Unitree L2 串口：

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

检查 UM981 串口：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

在真实 Ubuntu 上，之前识别到的是：

Unitree Lidar：

```text
/dev/ttyACM0
```

对应稳定路径：

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_593A032669-if00
```

UM981 开发板：

```text
/dev/ttyUSB0
```

对应稳定路径：

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

## 构建步骤

建议按依赖顺序构建并 source：

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

运行 launch 前建议 source：

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

## Stage 5：建图与定位导航

Stage 5 只有两个阶段：先建图并保存离线地图，再加载该地图进行定位和导航。每个新终端先执行上文的 `source` 命令。

### 阶段一：建图

室内没有卫星信号时，使用 fake RTK：

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

室外有有效 GNSS/RTK 定位时，将下面参数替换到同一条命令中：

```bash
use_um981:=true \
um981_port:=/dev/ttyUSB0 \
use_fake_rtk:=false
```

每次启动建图都会创建一个以启动时间命名的目录，例如：

```text
golf_mower_bringup/maps/stage5_segmented/20260710_142530/
```

建图完成后确认该目录中存在以下文件，然后按 `Ctrl+C` 停止：

```text
ground_map.pcd
nonground_map.pcd
map_metadata.yaml
```

### 阶段二：加载地图并定位导航

打开新终端，重新执行上文的 `source` 命令。室内使用 fake RTK：

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

室外有有效 GNSS/RTK 定位时，将下面参数替换到同一条命令中：

```bash
use_um981:=true \
um981_port:=/dev/ttyUSB0 \
use_fake_rtk:=false
```

#### 定位导航启动与地图处理

导航默认自动加载 `stage5_segmented` 下最新且同时包含 `ground_map.pcd`、`nonground_map.pcd` 和 `map_metadata.yaml` 的时间戳目录。如需加载指定历史地图，可显式设置 `segmented_map_dir` 和 `map_metadata_path`。

导航启动前会读取 `ground_map.pcd` 的 PCD 头部。若点数为 0 且 `allow_offline_ground_recovery:=true`，程序会从 `nonground_map.pcd` 提取与建图起点连通的局部最低地表，生成 `recovered_ground_map.pcd`、`recovered_nonground_map.pcd` 和 `recovery_report.yaml` 后再启动。恢复质量不合格、文件缺失或格式无效时仍会报错退出，原始 PCD 不会被覆盖。

fake RTK 只用于验证离线地图加载、定位接口和 TF 链路，不代表真实 GNSS 精度。室内导航测试时，机器人应尽量放回建图起点并保持相同朝向；否则需要调整 `rtk_map_to_camera_init_yaw`。

检查地图定位链路：

```bash
ros2 topic hz /fix
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /map --once
ros2 topic echo /cmd_vel
```

#### `/cmd_vel` 输出说明

`/cmd_vel` 是 Nav2 输出的车体速度目标，消息类型为 `geometry_msgs/msg/Twist`，不是电机转速、编码器数据或机器人实际速度。本项目的差速底盘主要使用以下字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `linear.x` | m/s | 车体前后线速度；正值前进，负值后退。 |
| `angular.z` | rad/s | 车体绕 Z 轴的角速度；正值左转，负值右转。 |

`linear.y`、`linear.z`、`angular.x` 和 `angular.y` 对普通差速底盘应保持为 0。全零消息表示停车目标。例如：

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

该消息表示机器人以 `0.30 m/s` 前进，同时以 `0.20 rad/s` 向左转。真实底盘驱动需要订阅 `/cmd_vel`，根据底盘运动学将其换算为左右轮目标速度，再通过底盘控制器协议发送给电机；同时必须实现速度限制、命令超时停车、通信故障停车和硬件急停。当前仓库还没有实现这个真实底盘驱动，因此能够观察到 `/cmd_vel` 不代表物理车轮会运动。

### Stage 5 参数说明

| 参数 | 示例值 | 简单说明 |
| --- | --- | --- |
| `initialize_type` | `1` | 雷达接口；`1` 为 USB 串口，`2` 为 UDP。 |
| `work_mode` | `8` | Unitree L2 串口工作模式。 |
| `serial_port` | `/dev/ttyACM0` | Unitree L2 串口，不是 UM981 端口。 |
| `baudrate` | `4000000` | Unitree L2 串口波特率。 |
| `start_lidar_rotation` | `true` | launch 启动时让雷达开始旋转。 |
| `reset_lidar_after_set_mode` | `false` | 设置模式后是否复位雷达；当前硬件使用 `false`。 |
| `patchwork_sensor_height` | `0.75` | 实测雷达中心离地高度，单位米。 |
| `lidar_tf_z` | `0.0` | 雷达相对 `base_link` 的 Z 方向安装偏移，单位米。 |
| `imu_tf_z` | `0.00667` | Unitree 内部 IMU 的 Z 方向安装偏移，单位米。 |
| `imu_quaternion_order` | `wxyz` | Unitree SDK IMU 四元数的排列顺序。 |
| `use_um981` | 室内 `false`，室外 `true` | 是否启动真实 UM981 GNSS 节点。 |
| `um981_port` | `/dev/ttyUSB0` | UM981 串口，仅在 `use_um981:=true` 时使用。 |
| `use_fake_rtk` | 室内 `true` | 从 Point-LIO 里程计模拟 `/fix`，仅用于室内测试。 |
| `use_map_metadata_recorder` | 建图 `true` | 将定位基准和建图起始信息写入 `map_metadata.yaml`。 |
| `map_metadata_path` | 默认 Stage 5 路径 | 建图写入、导航读取的地图定位元数据文件。 |
| `fix_topic` | `/fix` | UM981 或 fake RTK 发布 `NavSatFix` 的话题。 |
| `yaw_map_to_enu` | `0.0` | 地图 X 轴相对 ENU 东轴的角度，单位弧度。 |
| `use_pointlio_diagnostics` | 调试 `true` | 输出点云、IMU时间和里程计跳变诊断。 |
| `launch_rviz` | `true` | 建图时是否启动 RViz。 |
| `use_rtk_map_localizer` | 导航 `true` | 根据 `/fix` 和地图元数据发布 `map -> camera_init`。 |
| `allow_offline_ground_recovery` | `true` | ground为空时尝试从nonground恢复连通地面；恢复质量不合格则终止导航。 |
| `use_map_to_camera_init_adapter` | `false` | RTK定位器启用时关闭静态同名 TF，避免冲突。 |
| `use_um981_heading` | 当前 `false` | 是否使用 UM981 航向；当前没有有效 INS heading。 |
| `rtk_map_to_camera_init_yaw` | `0.0` | 不使用 UM981 heading 时的手动初始偏航角，单位弧度。 |
| `launch_outdoor_rviz` | `true` | 定位导航时是否启动 RViz。 |
| `use_coverage_planner` | 默认 `false` | 是否在Stage5导航中启动Fields2Cover ROS2节点。 |
| `coverage_area_file` | 示例YAML路径 | `map`坐标系下的作业边界、禁入区和机器人规划参数。 |
| `coverage_output_file` | `~/.ros/golf_mower/coverage_path.yaml` | 保存生成路径的位置。 |
| `coverage_dry_run` | 当前 `true` | 只规划、发布和保存，不向Nav2提交执行目标。 |
| `coverage_path_pose_spacing` | `0.10` | 发布/保存路径的采样间距，单位米。 |
| `coverage_nav_waypoint_spacing` | `0.75` | 提交给Nav2的航点采样间距，单位米。 |

布尔参数中，`true` 表示启用，`false` 表示禁用。角度使用弧度，位置和高度使用米。

## 覆盖路径规划

覆盖规划节点读取 `golf_mower_bringup/config/coverage_test_area.yaml`，在 `map` 坐标系中生成割草条带和转弯路径。

终端A启动规划节点并保持运行：

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

节点启动后自动规划并在RViz显示，无需调用 `replan`。

主要接口：

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/coverage_path` | `nav_msgs/msg/Path` | `map` 坐标系下的连续覆盖路径。 |
| `/coverage_markers` | `visualization_msgs/msg/MarkerArray` | 作业边界、禁入区和覆盖路径的RViz显示。 |
| `/coverage_planner/replan` | `std_srvs/srv/Trigger` | 重新读取YAML并规划。 |
| `/coverage_planner/execute` | `std_srvs/srv/Trigger` | 向Nav2提交航点；`dry_run=true` 时拒绝执行。 |

修改YAML后，在终端B重新规划：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source Fields2Cover-main/install/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 service call /coverage_planner/replan std_srvs/srv/Trigger '{}'
```

`'{}'` 是 `std_srvs/srv/Trigger` 的空请求，应原样保留。

Stage5第二阶段可增加：

```bash
use_coverage_planner:=true \
coverage_area_file:=/home/ubuntu/unilidar_sdk2/golf_mower_bringup/config/coverage_test_area.yaml \
coverage_dry_run:=true
```

YAML核心参数：

| YAML参数 | 单位 | 说明 |
| --- | --- | --- |
| `robot.width` | m | 机器人底盘物理宽度，Fields2Cover用于计算地头空间。 |
| `robot.coverage_width` | m | 刀盘单次通过的有效割草宽度，决定相邻条带间距。 |
| `robot.min_turning_radius` | m | 机器人中心轨迹允许的最小转弯半径。 |
| `robot.cruise_speed` | m/s | 直线割草条带的期望速度，当前仅写入Fields2Cover路径属性。 |
| `robot.turn_speed` | m/s | 条带连接和转弯段的期望速度，当前仅写入Fields2Cover路径属性。 |
| `planner.headland_swaths` | 条 | 边界内为转弯预留的地头条带数量。 |
| `planner.swath_angle_deg` | degree | 条带方向；设为 `null` 时由Fields2Cover自动选择。 |

`planner.headland_swaths` 用机器人宽度作为单位，表示边界内侧需要留出多宽的转弯区域，不表示机器人实际绕边界行驶多少圈：

```text
┌──────────────────────────────┐  作业区域边界 boundary
│        地头转弯区域           │
│    ┌────────────────────┐    │
│    │  → → → → → → → →  │    │
│    │  ← ← ← ← ← ← ← ←  │    │  中间：往返割草条带
│    │  → → → → → → → →  │    │
│    └────────────────────┘    │
│        地头转弯区域           │
└──────────────────────────────┘
```

近似预留宽度为 `robot.width × planner.headland_swaths`。例如底盘宽度为 `0.5 m`、`headland_swaths: 10` 时，边界内侧约预留 `5 m` 用于连接相邻割草条带和转弯。数值过小可能导致转弯轨迹越界；数值过大会缩小中间的有效割草区域。

`boundary` 和 `exclusions` 坐标单位为米，必须与离线地图的 `map` 坐标一致。`cruise_speed` 和 `turn_speed` 当前不会改变Nav2速度。底盘和安全系统完成前保持 `dry_run:=true`。

### 从点云生成候选边界

建图完成后，可以从地面点云的最大连通区域生成凸包候选边界。将 `<地图目录>` 替换为实际时间戳目录：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 run golf_mower_bringup coverage_boundary_from_pcd.py \
  --ground-pcd <地图目录>/ground_map.pcd \
  --output <地图目录>/coverage_candidate.yaml \
  --resolution 0.25 \
  --connect-gap-cells 2 \
  --robot-width 1.50 \
  --coverage-width 1.00 \
  --min-turning-radius 1.0 \
  --headland-swaths 3
```

`resolution` 是地面栅格大小；`connect-gap-cells` 用于连接点云中的小扫描间隙。输出的 `coverage_candidate.yaml` 可以直接作为 `coverage_area_file` 进行 `dry_run` 显示。该边界是最大连通地面的**凸包候选**，可能包含凹口、未扫描空洞、道路或不允许割草的区域，因此必须在RViz中确认并手动补充 `exclusions`，不得未经确认直接控制机器人。

若 `ground_map.pcd` 为空，不能生成候选边界。先在Stage5第二阶段完成离线地面恢复，再将输入改为同一目录下的 `recovered_ground_map.pcd`。

### Stage 5与覆盖规划的完整流程

启动第二阶段前需要准备：

1. 第一阶段生成的完整时间戳地图目录，其中包含 `ground_map.pcd`、`nonground_map.pcd` 和 `map_metadata.yaml`。默认自动选择最新目录，也可以通过 `segmented_map_dir` 和 `map_metadata_path` 指定。
2. `coverage_test_area.yaml`。其中 `boundary` 和 `exclusions` 必须使用离线地图的 `map` 坐标，点云地图不会自动生成割草边界；同时填写真实底盘宽度、割草宽度和最小转弯半径。
3. 室内测试时，将机器人放回建图起点附近并保持相同朝向，使用 fake RTK；室外运行时准备有效 UM981 定位和正确的地图航向关系。

第二阶段的处理顺序为：

```text
加载 ground/nonground PCD
  -> 生成 Nav2 使用的 /map
  -> RTK 或 fake RTK 确定 map -> camera_init
  -> Point-LIO 提供实时局部位姿
  -> Fields2Cover 读取 coverage_test_area.yaml
  -> 发布覆盖路径
  -> Nav2 跟踪覆盖航点
  -> 输出 /cmd_vel 给底盘驱动
```

首次联调保持 `coverage_dry_run:=true`。此时节点启动后会自动规划，可以在RViz检查边界、禁入区和路径，但不会让Nav2执行。确认路径和定位正确后，将启动参数改为 `coverage_dry_run:=false`，重新启动第二阶段，再在另一个已source的终端提交路径：

```bash
ros2 service call /coverage_planner/execute std_srvs/srv/Trigger '{}'
```

主要输出为：

| 输出 | 用途 |
| --- | --- |
| `/map` | 由离线地面/非地面点云生成的Nav2占据栅格地图。 |
| `/coverage_path` | `map`坐标系下的完整连续覆盖路径。 |
| `/coverage_markers` | RViz中的边界、禁入区和路径。 |
| `~/.ros/golf_mower/coverage_path.yaml` | 保存的覆盖路径文件，可由 `coverage_output_file` 修改。 |
| Nav2 `NavigateThroughPoses`目标 | 从覆盖路径抽样得到的导航航点。 |
| `/cmd_vel` | Nav2生成的车体速度目标，供底盘驱动执行。 |

当前仓库尚未接入真实底盘驱动，因此可以完成离线地图加载、定位、覆盖规划和Nav2速度输出测试，但机器人不会仅凭 `/cmd_vel` 自动驱动车轮。

### 障碍物处理与待完成任务

Fields2Cover负责决定“哪里需要割草”，Nav2负责安全跟踪路径。两者不直接冲突，但当前系统还不能自动恢复因绕障造成的漏割区域：

| 障碍类型 | 处理方式 |
| --- | --- |
| 树木、建筑、沙坑、水池等固定障碍 | 写入 `coverage_test_area.yaml` 的 `exclusions`，由Fields2Cover生成绕开障碍的覆盖路径。 |
| 行人、车辆、临时设备等动态障碍 | 由Nav2局部代价地图减速、停车或局部绕行。 |
| 长时间阻塞 | 当前Nav2任务可能失败，需要后续任务管理器跳过该段并安排补割。 |

真实无人割草前还需要完成：

1. 将完整覆盖路径拆分为可独立执行的条带或路径段。
2. 增加覆盖任务管理器，记录 `PENDING`、`ACTIVE`、`COMPLETED`、`BLOCKED` 和 `RETRY` 状态。
3. 短时障碍采用等待或Nav2局部绕行；超时后取消当前段、标记未完成并继续下一段。
4. 根据机器人实际轨迹和刀盘宽度维护已割覆盖栅格，而不是仅根据规划路径判断完成情况。
5. 任务结束或障碍消失后，从未覆盖栅格生成补割区域，并重新调用Fields2Cover规划。
6. 增加越界、定位失效、代价地图失效、通信超时和急停条件下的安全停车。

在上述任务管理与安全机制完成前，应保持 `coverage_dry_run:=true`，或只在封闭测试区域低速验证Nav2路径跟踪。

## 早期调试 Stage

Stage 1/2/3/4 用于逐层排查算法链路。Stage 5 能运行后，它们不是完整系统的首选入口。

### 机器人模型

```bash
ros2 launch golf_mower_description description.launch.py launch_rviz:=true
```

### 室内 Point-LIO 测试

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

检查：

```bash
ros2 topic hz /unilidar/cloud
ros2 topic hz /unilidar/imu
ros2 topic hz /pointlio/odom
```

### Stage 1：室外高程建图

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

检查：

```bash
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /elevation_mapping_node/elevation_map_filter
ros2 run golf_mower_bringup grid_map_inspect.py
```

### Stage 2：可通行性栅格

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

检查：

```bash
ros2 topic hz /elevation/traversability_grid
ros2 topic echo /elevation/traversability_grid --once
```

### Stage 3：基于高程栅格的 Nav2

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

### Stage 4：Patchwork++ 地面分割

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

检查：

```bash
ros2 topic hz /ground_segmentation/ground
ros2 topic hz /ground_segmentation/nonground
```

## 参数说明

### 雷达驱动参数

| 参数 | 常用值 | 说明 |
| --- | --- | --- |
| `initialize_type` | `1` 串口，`2` UDP | Unitree L2 初始化方式。USB 串口通常用 `1`。 |
| `work_mode` | `8` 串口，`0` UDP | Unitree L2 工作模式。串口 L2 测试常用 `8`。 |
| `serial_port` | `/dev/ttyACM0` | Unitree L2 串口设备。 |
| `baudrate` | `4000000` | Unitree L2 串口波特率。 |
| `start_lidar_rotation` | `true` | launch 时启动雷达旋转。 |
| `reset_lidar_after_set_mode` | `false` | 设置模式后是否重置雷达。如果重置导致启动不稳定，使用 `false`。 |
| `use_system_timestamp` | `false` | 是否使用主机时间代替传感器时间。传感器时间正常时建议 `false`。 |

### Point-LIO 和 IMU 参数

| 参数 | 常用值 | 说明 |
| --- | --- | --- |
| `pointlio_config_file` | `unilidar_l2_ros2_no_pcd.yaml` | Point-LIO 配置文件。 |
| `imu_quaternion_order` | `wxyz` | Unitree IMU 适配器使用的四元数字段顺序。 |
| `imu_angular_velocity_scale` | `0.017453292519943295` | 将 deg/s 转换为 rad/s。如果驱动已经输出 rad/s，才改为 `1.0`。 |
| `imu_linear_acceleration_scale` | `1.0` | 线加速度缩放系数。 |
| `use_static_pointlio_pose` | `false` | 测试时使用固定 pose，而不是实时 Point-LIO 输出。 |

### TF 和传感器安装参数

| 参数 | 说明 |
| --- | --- |
| `lidar_tf_x/y/z` | 雷达相对 `base_link` 的安装位置，单位米。 |
| `lidar_tf_roll/pitch/yaw` | 雷达相对 `base_link` 的安装姿态，单位弧度。 |
| `imu_tf_x/y/z` | IMU 相对 `base_link` 的安装位置，单位米。 |
| `imu_tf_roll/pitch/yaw` | IMU 相对 `base_link` 的安装姿态，单位弧度。 |
| `map_to_camera_init_x/y/z` | 手动静态 `map -> camera_init` 平移。 |
| `map_to_camera_init_roll/pitch/yaw` | 手动静态 `map -> camera_init` 旋转，单位弧度。 |
| `use_map_to_camera_init_adapter` | 是否发布静态 `map -> camera_init`。当 `use_rtk_map_localizer:=true` 时必须设为 `false`，避免 TF 冲突。 |

### Patchwork++ 参数

| 参数 | 常用值 | 说明 |
| --- | --- | --- |
| `patchwork_cloud_topic` | `/unilidar/cloud` | 地面分割输入点云。 |
| `patchwork_sensor_height` | `0.75` | 实测雷达离地高度。室外不要长期设为 `0`。 |
| `patchwork_min_r` | `0.2` | Patchwork++ 使用的最小半径。 |
| `patchwork_max_r` | `40.0` | Patchwork++ 使用的最大半径。 |
| `patchwork_log_every_n` | `60` | Patchwork++ 日志输出间隔。 |
| `min_ground_points` | `100` | 实时高程输入采用 Patchwork++ ground 前要求的最少点数。ground短时异常时冻结上一份高程图；连续异常后仅提取原始点云的局部最低表面，不再把整帧原始点云当地面。 |

### UM981 ROS Topic

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/fix` | `sensor_msgs/NavSatFix` | GGA、INSPVAX 或 DRPVA 解析出的 GNSS/RTK 位置。 |
| `/imu/data_raw` | `sensor_msgs/Imu` | 当前机器人流程不使用；实测 UM981 USB 输出没有 RAWIMUX。 |
| `/um981/heading` | `std_msgs/Float64` | 当前机器人流程不使用；预留给有效 INS heading 输出。 |
| `/um981/ins_attitude` | `geometry_msgs/Vector3Stamped` | 当前机器人流程不使用。 |

## 调试建议

Point-LIO 初始化时，机器人和雷达应静止 5 到 10 秒。启动瞬间移动、碰撞雷达或拉扯线缆都可能导致初始 IMU bias 和重力估计错误，表现为点云地图漂移。

常见问题：

- `Patchwork++ produced empty ground cloud`：检查 `patchwork_sensor_height` 和点云坐标轴方向。
- `Lookup would require extrapolation into the future`：TF 发布时间略慢于点云时间，可能影响分割地图累计，不一定代表 Point-LIO 本身漂移。
- `/ground_segmentation/ground` 一直为空：不要长期使用 `patchwork_sensor_height:=0`，应填写雷达真实离地高度。
- `/fix` 是 `STATUS_NO_FIX`：UM981 在室内或没有有效卫星定位。室内用 fake RTK，真实 GNSS 到室外测试。

## 后续开发方向

- 等当前串口能输出有效 heading 后，再启用并标定 UM981 INS heading。
- 标定 RTK 天线杆臂。
- 使用 `robot_localization` 融合 RTK、轮速、IMU 和 Point-LIO。
- 根据真实球场边界、禁入区和割草宽度标定覆盖规划参数。
- 在底盘安全闭环完成后验证覆盖路径的Nav2实车执行。
- 加入割草机构状态、急停、边界保护和禁入区。
