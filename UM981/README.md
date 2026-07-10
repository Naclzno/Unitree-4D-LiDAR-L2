# UM981 Python SDK and ROS2 Driver

**重要：测试前请先将天线置于户外一段时间。IMU/INS 数据可能需要设备完成基本启动或对准后才稳定输出；position 定位数据不一定会立刻获得。即使在卫星信号较好的条件下，解算出可用的经纬度和高度也可能需要约 10 分钟。**

这个仓库用于读取 UM981 的定位坐标数据和 IMU 数据，并发布到 ROS2。

当前结构：

```text
.
├── docs/                 # UM981 协议资料
├── logs/                 # 运行日志，按 position/imu 分目录保存
├── resource/             # ROS2 ament 资源索引
├── tools/                # 本地调试工具
├── um981/                # 纯 Python SDK，不依赖 ROS
├── um981_ros/            # ROS2 节点
├── package.xml
├── setup.cfg
└── setup.py
```

推荐流程：

```text
WSL2 Ubuntu22.04 测试 SDK -> 确认串口/解析正常 -> 搬到真实 Ubuntu22.04 ROS2 主机 -> 发布 ROS topic
```

## WSL2 测试 SDK

先检查是否已有串口依赖：

```bash
python3 -c "import serial; print(serial.__version__)"
```

如果提示 `No module named 'serial'`，再安装：

```bash
sudo apt install python3-serial
```

在 WSL2 中先查看串口设备：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyS*
```

如果提示没有这些文件，说明 WSL2 还没有拿到 USB 串口设备。需要在 Windows PowerShell 中用 `usbipd-win` 把 UM981 对应的 USB 串口设备挂进 WSL2：

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

本机测试时 UM981 开发板可能显示为 CH340/CH341 USB 串口，例如：

```powershell
usbipd list
```

```text
Connected:
BUSID  VID:PID    DEVICE                   STATE
1-1    1a86:7523  USB-SERIAL CH340 (COM5)  Shared
```

这里的 `BUSID` 是 `1-1`。如果状态已经是 `Shared`，说明 `bind` 已经做过，通常只需要执行：

```powershell
usbipd attach --wsl --busid 1-1
```

Windows 重启、WSL2 重启或设备重新插拔后，通常需要重新执行 `attach`。简单区分：

- `usbipd bind --busid 1-1`：一般只需要做一次，或状态不是 `Shared` 时再做。
- `usbipd attach --wsl --busid 1-1`：每次重启电脑、重启 WSL2、重新插拔 UM981 后都可能需要再做一次。

然后回到 WSL2 检查：

```bash
lsusb
dmesg | tail
ls -l /dev/ttyUSB* /dev/ttyACM*
```

挂载成功后，常见设备名是 `/dev/ttyUSB0` 或 `/dev/ttyACM0`，不一定是 `/dev/ttyS5`。

如果只看到 `/dev/ttyS0` 到 `/dev/ttyS3`，通常只代表 WSL2 暴露了 COM1 到 COM4；Windows 上的 `COM5` 仍然没有进入 WSL2。此时不要直接试 `/dev/ttyS5`，优先使用 `usbipd-win` 挂载 USB 串口设备，或在 Windows 设备管理器中把 UM981 的端口号临时改到 COM1 到 COM4 之一。

如果串口权限不足，把当前用户加入 `dialout` 组，然后重新打开 WSL：

```bash
sudo usermod -aG dialout $USER
```

运行 SDK 测试工具：

```bash
python3 tools/um981_echo.py --port /dev/ttyUSB0 --baud 115200
```

默认会一直运行，直到按 `Ctrl+C` 停止。也可以用 `--duration` 指定采集时长，单位是秒：

```bash
python3 tools/um981_echo.py --port /dev/ttyUSB0 --duration 60
```

上面命令表示采集 60 秒后自动退出。

默认发送：

- `RAWIMUXA 0.01`：100 Hz 原始 IMU。
- `IMUATTA 0.1`：10 Hz INS/IMU 姿态和 IMU 测量值，作为部分设备不输出 `RAWIMUXA` 时的备用 IMU 来源。
- `INSPVAXA 0.1`：10 Hz INS 组合导航位置、速度、姿态。

只看定位坐标：

```bash
python3 tools/um981_echo.py --port /dev/ttyUSB0 --type position
```

只看 IMU：

```bash
python3 tools/um981_echo.py --port /dev/ttyUSB0 --type imu
```

临时调整输出频率：

```bash
python3 tools/um981_echo.py \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --command "RAWIMUXA 0.02"
```

输出为 JSON Lines，只保留两类 observation：

| 类型 | 来源 | 含义 |
| --- | --- | --- |
| `position` | `$GNGGA`，或有效的 `INSPVAX/DRPVA` | 经纬度、高度、定位质量 |
| `imu` | `RAWIMUX` 或 `IMUATT` | 三轴加速度和三轴角速度 |

如果设备没有输出 `$GNGGA`，请先用 UM981 配置工具或串口命令打开 NMEA GGA 输出。

每次运行都会在 `logs/positions/` 和 `logs/imus/` 下分别保存按时间命名的 `.txt` 文件，例如：

```text
logs/positions/um981_position_20260703_170530.txt
logs/imus/um981_imu_20260703_170530.txt
```

如果临时不想保存日志：

```bash
python3 tools/um981_echo.py --port /dev/ttyUSB0 --no-log
```

## 保存的 TXT 数据格式

`logs/` 下保存的是 JSON Lines 格式：每一行都是一条完整 JSON 记录，可以按行读取、筛选和回放。

目前只保存两类数据：

- `position`：定位坐标数据，包含经度、纬度和高度。
- `imu`：IMU 加速度和角速度数据。

文件保存位置：

| 路径 | 内容 |
| --- | --- |
| `logs/positions/um981_position_*.txt` | 只保存 `position` 定位坐标行 |
| `logs/imus/um981_imu_*.txt` | 只保存 `imu` IMU 行 |

定位数据一般频率低于 IMU，所以分开保存后，`positions` 文件会明显比 `imus` 文件小。

### 定位坐标行

示例：

```json
{"type": "position", "message": "GNGGA", "payload": {"lat_deg": 39.771899393, "lon_deg": 116.35297469283333, "height_m": 67.7774, "fix_quality": "gps_fix", "satellites_used": 12, "hdop": 2.2, "source": "GNGGA"}, "raw_text": "$GNGGA,..."}
```

字段说明：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `type` | 数据类型，定位坐标固定为 `position` | - |
| `message` | 原始消息类型，例如 `GNGGA` | - |
| `payload.lat_deg` | 纬度，北纬为正，南纬为负 | 度 |
| `payload.lon_deg` | 经度，东经为正，西经为负 | 度 |
| `payload.height_m` | 高度/海拔 | m |
| `payload.fix_quality` | 定位质量，例如 `gps_fix`、`rtk_fixed`、`rtk_float` | - |
| `payload.satellites_used` | 参与定位解算的卫星数量 | 颗 |
| `payload.hdop` | 水平精度因子，越小通常越好 | - |
| `raw_text` | UM981 原始串口文本 | - |

### IMU 行

示例：

```json
{"type": "imu", "message": "RAWIMUXA", "payload": {"week": 2425, "seconds": 467780.9, "accel_mps2": {"x": 7.243878188421277, "y": 6.296942533646656, "z": 0.7972934842982268}, "gyro_radps": {"x": 0.026632423657862017, "y": -0.0316925841528558, "z": 0.006924430151044124}, "imu_error": "00", "imu_type": 64, "imu_status": "0f200000", "raw_counts": [1332, -10520, 12102, 52, 238, 200]}, "raw_text": "#RAWIMUXA,..."}
```

字段说明：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `type` | 数据类型，IMU 固定为 `imu` | - |
| `message` | 原始消息类型，例如 `RAWIMUXA` | - |
| `payload.week` | GNSS 周 | - |
| `payload.seconds` | GNSS 周内秒 | s |
| `payload.accel_mps2.x/y/z` | 三轴加速度 | m/s^2 |
| `payload.gyro_radps.x/y/z` | 三轴角速度 | rad/s |
| `payload.imu_error` | IMU 错误标志，`00` 通常表示正常 | - |
| `payload.imu_type` | IMU 类型编号 | - |
| `payload.imu_status` | IMU 状态字 | - |
| `payload.raw_counts` | RAWIMUX 原始计数值 | - |
| `raw_text` | UM981 原始串口文本 | - |

## ROS2 主机使用

在真实 Ubuntu22.04 ROS2 主机上，把本仓库放进工作空间：

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/UM981 ~/ros2_ws/src/um981_ros
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select um981_ros
source install/setup.bash
```

连接 UM981 后确认串口名，常见为 `/dev/ttyUSB0`、`/dev/ttyACM0` 或 `/dev/ttyS*`。

启动 ROS2 节点：

```bash
ros2 run um981_ros um981_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=115200 \
  -p frame_id:=gnss_link \
  -p imu_frame_id:=imu_link
```

检查 topic：

```bash
ros2 topic list
ros2 topic echo /fix
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

当前发布：

| Topic | ROS 类型 | 数据来源 |
| --- | --- | --- |
| `/fix` | `sensor_msgs/NavSatFix` | `$GNGGA`，以及有效的 `INSPVAX/DRPVA` |
| `/imu/data_raw` | `sensor_msgs/Imu` | `RAWIMUX`，或备用的 `IMUATT` |
| `/um981/heading` | `std_msgs/Float64` | `INSPVAX.azimuth_deg` 或 `DRPVA.heading_deg`，单位为度，北偏东/顺时针为正 |
| `/um981/ins_attitude` | `geometry_msgs/Vector3Stamped` | `x=roll_deg`，`y=pitch_deg`，`z=heading_deg` |

## 串口识别和调试

在真实 Ubuntu 主机上，如果同时连接 Unitree Lidar 和 UM981，不要只看 `/dev/ttyACM0` 或 `/dev/ttyUSB0` 的名字；这些名字会随插入顺序变化。建议使用稳定的 by-id 路径：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null
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

测试 UM981 时建议优先使用：

```bash
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

本项目提供了插拔对比脚本，用于找出 UM981 对应的新增串口：

```bash
cd /home/ubuntu/unilidar_sdk2/UM981
python3 tools/find_um981_port.py
```

脚本会先记录未插 UM981 时的串口设备，然后提示插入 UM981，再列出新增的 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。

如果 UM981 使用 CH340/CH341 USB 串口芯片，在 Ubuntu 上可能被 `brltty` 服务误抢，表现为 `ttyUSB0` 刚出现又断开。`dmesg` 中可能出现：

```text
ch341-uart converter now attached to ttyUSB0
usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
ch341-uart ttyUSB0: ch341-uart converter now disconnected from ttyUSB0
```

如果不使用盲文显示器，可以禁用 `brltty`：

```bash
sudo systemctl stop brltty
sudo systemctl disable brltty
```

更彻底的方式是卸载：

```bash
sudo apt remove brltty
```

确认串口后，可以用 probe 工具检查原始输出：

```bash
cd /home/ubuntu/unilidar_sdk2/UM981
python3 tools/um981_probe.py --port /dev/ttyUSB0 --baud 115200 --listen 10
```

正常应至少能看到 `$GNGGA`。如果要使用 IMU 或 heading，还需要看到以下任一类原始帧：

```text
#RAWIMUXA
#IMUATTA
#INSPVAXA
#DRPVAA
```

如果命令返回 `response: OK`，但持续监听只有 `$GNGGA`，没有 `#RAWIMUXA/#IMUATTA/#INSPVAXA`，说明当前 USB 串口只输出 NMEA/GGA，IMU/INS 数据可能没有映射到这个端口，需要通过厂商配置工具或开发板其他 UART 口打开对应输出。

## 坐标系说明

`RAWIMUX` 原始字段顺序为 `z, -y, x`。SDK 当前先转换为 `x, y, z`，再发布到 ROS IMU。`IMUATT` 已按协议中的 X/Y/Z 加速度和角速度字段解析，作为备用 IMU 来源。

上车或接入融合算法前，建议做三项检查：

1. 静置时加速度方向是否符合 `imu_link` 定义。
2. 绕 X/Y/Z 轴转动时角速度正负号是否符合 ROS 坐标系。
3. `frame_id`、`imu_frame_id` 是否和机器人 TF 树一致。

## 代码入口

- SDK 串口入口：[um981/sdk.py](/home/yxy/UM981/um981/sdk.py)
- 协议解析：[um981/protocol.py](/home/yxy/UM981/um981/protocol.py)
- WSL2 调试工具：[tools/um981_echo.py](/home/yxy/UM981/tools/um981_echo.py)
- ROS2 节点：[um981_ros/node.py](/home/yxy/UM981/um981_ros/node.py)
