import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name, default_value, description=''):
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def generate_launch_description():
    description_share = get_package_share_directory('golf_mower_description')
    default_model = os.path.join(description_share, 'urdf', 'golf_mower.urdf.xacro')
    default_rviz = os.path.join(description_share, 'rviz', 'description.rviz')

    model_arg = _arg('model', default_model, 'Path to the golf mower xacro file.')
    use_sim_time_arg = _arg('use_sim_time', 'false')
    launch_rviz_arg = _arg('launch_rviz', 'false')
    rviz_config_arg = _arg('rviz_config', default_rviz)

    xacro_args = [
        _arg('base_length', '1.30'),
        _arg('base_width', '0.90'),
        _arg('base_height', '0.30'),
        _arg('base_z', '0.25'),
        _arg('lidar_x', '0.35'),
        _arg('lidar_y', '0.0'),
        _arg('lidar_z', '0.65'),
        _arg('lidar_roll', '0.0'),
        _arg('lidar_pitch', '0.0'),
        _arg('lidar_yaw', '0.0'),
        _arg('imu_x', '0.342302'),
        _arg('imu_y', '-0.014655'),
        _arg('imu_z', '0.65667'),
        _arg('imu_roll', '0.0'),
        _arg('imu_pitch', '0.0'),
        _arg('imu_yaw', '0.0'),
        _arg('rtk_x', '0.0'),
        _arg('rtk_y', '0.0'),
        _arg('rtk_z', '1.00'),
        _arg('rtk_roll', '0.0'),
        _arg('rtk_pitch', '0.0'),
        _arg('rtk_yaw', '0.0'),
        _arg('mower_x', '-0.15'),
        _arg('mower_y', '0.0'),
        _arg('mower_z', '-0.10'),
        _arg('mower_roll', '0.0'),
        _arg('mower_pitch', '0.0'),
        _arg('mower_yaw', '0.0'),
        _arg('mower_width', '0.80'),
        _arg('mower_length', '0.35'),
        _arg('wheel_radius', '0.16'),
        _arg('wheel_width', '0.08'),
        _arg('wheel_x', '-0.20'),
        _arg('wheel_y', '0.48'),
        _arg('wheel_z', '0.0'),
    ]

    xacro_command = [
        'xacro ', LaunchConfiguration('model'),
        ' base_length:=', LaunchConfiguration('base_length'),
        ' base_width:=', LaunchConfiguration('base_width'),
        ' base_height:=', LaunchConfiguration('base_height'),
        ' base_z:=', LaunchConfiguration('base_z'),
        ' lidar_x:=', LaunchConfiguration('lidar_x'),
        ' lidar_y:=', LaunchConfiguration('lidar_y'),
        ' lidar_z:=', LaunchConfiguration('lidar_z'),
        ' lidar_roll:=', LaunchConfiguration('lidar_roll'),
        ' lidar_pitch:=', LaunchConfiguration('lidar_pitch'),
        ' lidar_yaw:=', LaunchConfiguration('lidar_yaw'),
        ' imu_x:=', LaunchConfiguration('imu_x'),
        ' imu_y:=', LaunchConfiguration('imu_y'),
        ' imu_z:=', LaunchConfiguration('imu_z'),
        ' imu_roll:=', LaunchConfiguration('imu_roll'),
        ' imu_pitch:=', LaunchConfiguration('imu_pitch'),
        ' imu_yaw:=', LaunchConfiguration('imu_yaw'),
        ' rtk_x:=', LaunchConfiguration('rtk_x'),
        ' rtk_y:=', LaunchConfiguration('rtk_y'),
        ' rtk_z:=', LaunchConfiguration('rtk_z'),
        ' rtk_roll:=', LaunchConfiguration('rtk_roll'),
        ' rtk_pitch:=', LaunchConfiguration('rtk_pitch'),
        ' rtk_yaw:=', LaunchConfiguration('rtk_yaw'),
        ' mower_x:=', LaunchConfiguration('mower_x'),
        ' mower_y:=', LaunchConfiguration('mower_y'),
        ' mower_z:=', LaunchConfiguration('mower_z'),
        ' mower_roll:=', LaunchConfiguration('mower_roll'),
        ' mower_pitch:=', LaunchConfiguration('mower_pitch'),
        ' mower_yaw:=', LaunchConfiguration('mower_yaw'),
        ' mower_width:=', LaunchConfiguration('mower_width'),
        ' mower_length:=', LaunchConfiguration('mower_length'),
        ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
        ' wheel_width:=', LaunchConfiguration('wheel_width'),
        ' wheel_x:=', LaunchConfiguration('wheel_x'),
        ' wheel_y:=', LaunchConfiguration('wheel_y'),
        ' wheel_z:=', LaunchConfiguration('wheel_z'),
    ]

    robot_description = {
        'robot_description': ParameterValue(Command(xacro_command), value_type=str)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_golf_mower_description',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        rviz_config_arg,
        *xacro_args,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
