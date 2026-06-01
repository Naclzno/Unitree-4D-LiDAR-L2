# golf_mower_description

URDF/Xacro description package for the golf mower robot.

The description publishes fixed sensor/tool frames from `base_link`:

- `unilidar_lidar`
- `unilidar_imu`
- `rtk_antenna`
- `mower_tool`
- `left_wheel`
- `right_wheel`

Run the description alone:

```bash
ros2 launch golf_mower_description description.launch.py launch_rviz:=true
```

Example with measured sensor offsets:

```bash
ros2 launch golf_mower_description description.launch.py \
  lidar_x:=0.35 lidar_y:=0.00 lidar_z:=0.65 \
  imu_x:=0.342302 imu_y:=-0.014655 imu_z:=0.65667 \
  rtk_x:=0.00 rtk_y:=0.00 rtk_z:=1.00 \
  mower_x:=-0.15 mower_y:=0.00 mower_z:=-0.10 \
  mower_width:=0.80 \
  launch_rviz:=true
```

The default Unitree IMU origin is set from the measured offset in the LiDAR
point cloud frame:

```text
lidar -> imu translation = [-0.007698, -0.014655, 0.00667] m
```

Use `base_link` as the robot body reference. The expected frame tree is:

```text
base_footprint -> base_link
base_link -> unilidar_lidar
base_link -> unilidar_imu
base_link -> rtk_antenna
base_link -> mower_tool
base_link -> left_wheel
base_link -> right_wheel
```

Dynamic localization frames are not published by this package. They should come from localization:

```text
map -> odom
odom -> base_link
base_link -> sensors/tools
```
