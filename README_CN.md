# 高尔夫除草机器人 ROS2 项目

[English](README.md) | 中文

这是一个面向高尔夫球场除草机器人的 ROS2 算法工程，集成 Unitree L2 激光雷达、Point-LIO、Patchwork++ 地面分割、高程/可通行性建图、Nav2 导航和 UM981 RTK/INS 定位。

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
- 将 `/cmd_vel` 转换为已文档化的电机串口协议；节点默认安全、需要显式解锁。
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
| 底盘电机驱动 | 原型已接入 | `motor_driver_node.py` 将经安全控制器后的速度指令映射为已文档化的离散命令，带超时制动、dry-run和显式解锁；串口参数与实车标定尚未验证。 |
| 轮速里程计 | 未实现 | 没有 `/wheel/odom` 数据源和经过标定的底盘运动学。 |
| 割草执行器 | 仅模型 | URDF包含 `mower_tool`，但没有刀盘电机接口、反馈和故障处理。 |
| 覆盖路径规划 | 已接入ROS2 | 从YAML读取 `map` 边界和禁入区，发布路径/Marker、保存结果，并提供可选Nav2 action；默认dry-run。 |
| 任务管理 | 基础版已实现 | 按完整条带/转弯序列逐段执行、重试、超时、阻塞跳过和取消；尚无实际覆盖记录与补割。 |
| 安全系统 | 初版软件门控已接入 | 安全控制器通过显式解锁、里程计新鲜度、命令新鲜度和锁存的软件急停门控`/cmd_vel`；硬件急停和刀盘互锁尚未接入。 |

当前仓库属于感知、建图、定位初始化和导航规划原型，还不是完整的自主除草机器人。实体运动、刀盘执行、覆盖作业和安全互锁完成并验证前，不能用于真实球场自动割草。

下一阶段的重点是验证底盘实物闭环：电机协议反馈、轮速里程计、有线硬件急停和受控低速测试。随后再完成连续状态估计、实车覆盖执行、割草机构和任务/安全状态机。

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
| `golf_mower_bringup` | 主要 launch、配置、诊断、地图工具、Nav2接入、覆盖规划、电机驱动和安全控制器。 |
| `motor` | 电机控制器 Serial V2 协议参考。 |
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

UM981 默认只接受 GGA 中质量为 `rtk_fixed`、卫星数不少于 10、HDOP 不大于 1.5 的数据作为可用 `/fix`。通过 `/um981/fix_quality` 查看拒绝原因。仅在受控诊断时才放宽 `um981_require_rtk_fixed`、`um981_allow_rtk_float`、`um981_min_satellites` 或 `um981_max_hdop`。

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
  use_coverage_geofence:=true \
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

Stage5 默认使用 CPU 高程建图（`use_cuda_elevation:=false`）。只有确认 NVIDIA 驱动和 GPU 运行环境正常后才启用 CUDA。地图目录默认是 `~/unilidar_sdk2/golf_mower_bringup/maps/stage5_segmented`；如需改用其他目录，在启动前设置 `GOLF_MOWER_MAP_ROOT`。

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

该消息表示机器人以 `0.30 m/s` 前进，同时以 `0.20 rad/s` 向左转。Stage5 中，安全控制器将该速度流转发到 `/motor/cmd_vel`；`motor_driver_node.py` 默认消费该话题，并使用 `motor/最新电机命令.md` 中的 Serial V2 协议。当前支持前进、后退、前进弧线、原地转向和制动；由于文档中的 `0x09` 差速命令没有定义有符号轮速方向，节点会主动拒绝倒车转向，而不会猜测编码。

#### 电机驱动

电机节点默认不启动。仅当运动命令或速度改变时才发送串口帧。零 `/cmd_vel`、`/cmd_vel` 超时、取消解锁、解锁完成和进程退出都会发送停止命令；默认使用文档中的 `0x04` 制动帧。`/motor_driver/status` 仅表示已下发的命令状态，不代表车轮真实运动。

先在不打开串口的情况下检查帧：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 launch golf_mower_bringup motor_driver.launch.py enabled:=true dry_run:=true
```

在另一个已 source 的终端发布测试目标并查看状态：

```bash
ros2 topic pub --once /motor/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.20}, angular: {z: 0.0}}'
ros2 topic echo /motor_driver/status
```

按保守默认限速，上述测试会打印 `AA 02 0A 13 C9 55`（19%速度）和 `AA 01 01 AC 55`（前进）；超时后打印 `AA 01 04 AF 55`。只有在驱动轮悬空测试完成后，才标定 `min_speed_percent` 和 `max_speed_percent`。

实车台架测试前，请将驱动轮悬空，并先确认控制器波特率、`8N1`/流控、最小命令间隔和硬件急停回路。节点以未解锁状态启动，之后显式解锁：

```bash
ros2 launch golf_mower_bringup motor_driver.launch.py \
  port:=<电机串口> \
  baudrate:=<已确认波特率> \
  dry_run:=false

ros2 service call /motor_driver/enable std_srvs/srv/SetBool '{data: true}'
```

解除解锁并制动：

```bash
ros2 service call /motor_driver/enable std_srvs/srv/SetBool '{data: false}'
```

#### 安全控制器

同时启动安全节点和电机节点时，Stage5 使用以下链路：

```text
/cmd_vel -> /safety_controller -> /motor/cmd_vel -> /motor_driver
```

安全控制器默认未解锁。只有显式解锁后，且`/pointlio/odom`和速度命令都在有效时间内，才会转发新的速度命令。向`/emergency_stop`发布`true`会立即发布零速度、锁存软件急停；输入恢复为`false`后仍需显式重置。

```bash
ros2 service call /motor_driver/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /safety_controller/enable std_srvs/srv/SetBool '{data: true}'

# 仅用于软件急停测试，不能替代硬件急停。
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'
ros2 service call /safety_controller/reset_emergency_stop std_srvs/srv/Trigger '{}'
```

在 Stage5 中，先增加 `use_motor_driver:=true use_safety_controller:=true motor_dry_run:=true` 做无输出联调。实车输出还需设置`motor_port`、`motor_baudrate`、`motor_dry_run:=false`、`use_coverage_geofence:=true`和`use_nav2:=true`；缺少其中任何条件时，launch 会报错退出。先解锁电机节点，再解锁安全控制器。在硬件急停和轮速里程计可用前，不能进行真实覆盖作业。

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
| `um981_require_rtk_fixed` | `true` | 仅接受 GGA 质量为 `rtk_fixed` 的数据作为 `/fix`。 |
| `um981_allow_rtk_float` | `false` | RTK fixed 不可用时是否允许 `rtk_float`；正常建图初始化不建议开启。 |
| `um981_min_satellites` | `10` | UM981 发布可用 `/fix` 前要求的最少卫星数。 |
| `um981_max_hdop` | `1.5` | 接受的最大 GGA HDOP。 |
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
| `use_coverage_geofence` | 覆盖任务时 `true` | 将同一份 `boundary/exclusions` 写入Nav2全局和局部代价地图，禁止局部绕障越界。 |
| `coverage_output_file` | `~/.ros/golf_mower/coverage_path.yaml` | 保存生成路径的位置。 |
| `coverage_mission_state_file` | `~/.ros/golf_mower/coverage_mission_state.yaml` | 保存每个任务段状态、重试次数和最近事件；仅用于诊断，不会自动续跑。 |
| `coverage_dry_run` | 当前 `true` | 只规划、发布和保存，不向Nav2提交执行目标。 |
| `coverage_path_pose_spacing` | `0.10` | 发布/保存路径的采样间距，单位米。 |
| `coverage_nav_waypoint_spacing` | `0.75` | 提交给Nav2的航点采样间距，单位米。 |
| `use_motor_driver` | 默认 `false` | 是否启动电机节点；默认消费 `/motor/cmd_vel`。 |
| `motor_port` | 空 | 电机控制器串口；dry-run测试可以保持为空。 |
| `motor_baudrate` | `115200` | 暂定电机串口波特率，必须按控制器资料确认。 |
| `motor_dry_run` | 默认 `true` | 仅打印串口帧；设为`false`后仍需要服务解锁才会输出。 |
| `motor_cmd_vel_timeout_sec` | `0.50` | 电机速度输入超过该时间未更新时发送配置的制动命令。 |
| `motor_max_linear_speed_mps` | `0.45` | 映射到最大电机百分比的`/cmd_vel.linear.x`绝对值。 |
| `motor_max_angular_speed_radps` | `0.70` | 映射到最大电机百分比的`/cmd_vel.angular.z`绝对值。 |
| `motor_min_speed_percent` | `10` | 最小非零命令使用的保守电机百分比。 |
| `motor_max_speed_percent` | `30` | 实车轮速标定前使用的保守最大电机百分比。 |
| `use_safety_controller` | 默认 `false` | 启动位于Nav2 `/cmd_vel`和`/motor/cmd_vel`之间的软件门控节点；物理电机输出时必须启用。 |
| `safety_emergency_stop_topic` | `/emergency_stop` | `std_msgs/Bool`；`true`锁存软件急停并发布零速度。 |
| `safety_cmd_vel_timeout_sec` | `0.50` | Nav2速度目标超过该时间未更新时，安全节点阻断输出。 |
| `safety_odom_timeout_sec` | `0.50` | `/pointlio/odom`超过该时间未更新时，安全节点阻断输出。 |

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
| `/coverage_planner/status` | `std_msgs/msg/String` | 当前任务段及完成、阻塞、取消数量。 |
| `/coverage_planner/replan` | `std_srvs/srv/Trigger` | 重新读取YAML并规划。 |
| `/coverage_planner/execute` | `std_srvs/srv/Trigger` | 启动逐段Nav2执行；`dry_run=true` 时拒绝执行。 |
| `/coverage_planner/cancel` | `std_srvs/srv/Trigger` | 取消当前目标和剩余覆盖任务。 |

修改YAML后，在终端B重新规划：

```bash
cd /home/ubuntu/unilidar_sdk2
source /opt/ros/humble/setup.bash
source Fields2Cover-main/install/setup.bash
source golf_mower_bringup/install/setup.bash

ros2 service call /coverage_planner/replan std_srvs/srv/Trigger '{}'
```

`'{}'` 是 `std_srvs/srv/Trigger` 的空请求，应原样保留。

上面的 Stage5 第二阶段命令已启用安全的覆盖规划 dry-run。仅在需要时调整 `coverage_mission_state_file`、`coverage_segment_max_waypoints`、`coverage_segment_max_retries` 或 `coverage_segment_timeout_sec`。`coverage_segment_max_waypoints` 仅用于告警，不会在完整割草条带或转弯中间切断；任务段最终失败时默认停止（`coverage_continue_after_blocked:=false`）。

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

### Stage 5 覆盖作业

启动 Stage5 第二阶段前，准备完整的时间戳地图目录和确认过的 `coverage_test_area.yaml`。其中 `boundary`、`exclusions` 使用离线地图的 `map` 坐标，并填写真实底盘宽度、割草宽度和最小转弯半径。室内测试时，机器人应回到建图起点附近并保持朝向一致，使用 fake RTK。

第二阶段的数据流为：

```text
离线 PCD -> Nav2 /map -> RTK 或 fake RTK 地图初始化
         -> Point-LIO 实时位姿 -> Fields2Cover 路径 -> Nav2 航点 -> /cmd_vel
```

首次联调使用 `coverage_dry_run:=true`：规划器发布 `/coverage_path`、`/coverage_markers`，保存 `coverage_output_file`，但不会运动。确认定位和路径后，以 `coverage_dry_run:=false` 重启，再显式启动任务：

```bash
ros2 service call /coverage_planner/execute std_srvs/srv/Trigger '{}'
```

任务管理器以完整割草条带和转弯序列为单位提交 Nav2，记录 `PENDING`、`ACTIVE`、`COMPLETED`、`BLOCKED`、`CANCELED`，并按配置重试或超时取消。查看状态或取消任务：

```bash
ros2 topic echo /coverage_planner/status
ros2 service call /coverage_planner/cancel std_srvs/srv/Trigger '{}'
```

Fields2Cover 处理 `exclusions` 中的固定障碍；Nav2 通过局部代价地图处理临时障碍。任务段最终阻塞时默认停止（`coverage_continue_after_blocked:=false`）。目前系统不会自动补割绕障遗漏区域，因此除受控低速测试外，应保持物理输出关闭。

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

## 通用参数说明

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
| `/um981/fix_quality` | `std_msgs/String` | UM981 解析出的定位质量、卫星数、HDOP 与接受结果。 |
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

## 后续开发任务

1. 接入常闭、切断电机电源的有线硬件急停，并将状态发布到`/emergency_stop`用于诊断；不能依赖ROS软件急停。
2. 确认电机控制器串口参数、ACK/NACK、内部命令看门狗、状态返回帧和带方向的差速轮速编码；随后补充反馈解析和可配置保活。
3. 实测完整车体外廓，并将 Fields2Cover 宽度、Nav2 footprint、膨胀半径和禁入区安全边距维护为同一份标定几何参数。
4. 接入轮速编码器，标定轮径和轮距，发布`/wheel/odom`，并用`robot_localization`融合轮速、RTK、IMU和Point-LIO。
5. 在室外根据接收机实际 GGA 输出验证 UM981 质量阈值；请求并验证`INSPVAX`或`DRPVA`后，再启用基于 heading 的初始化。
6. 接入 Nav2 碰撞监控，并为割草直线、转弯、障碍物附近和定位降级配置速度区；实车保持`coverage_continue_after_blocked:=false`。
7. 改进离线地图语义：增加离群点过滤、地图尺寸上限、地面密度检查和经人工确认的自由空间重建方法。
8. 根据刀盘真实状态和实测轨迹维护覆盖栅格，从绕障后的未覆盖区域生成补割任务。
9. 在实地运行前补充伪串口电机测试、launch测试、录制bag回归测试和硬件在环检查清单。
