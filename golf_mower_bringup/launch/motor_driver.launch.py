from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/motor/cmd_vel'),
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('require_fresh_cmd_after_arm', default_value='true'),
        DeclareLaunchArgument('command_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('cmd_vel_timeout_sec', default_value='0.50'),
        DeclareLaunchArgument('stop_mode', default_value='brake'),
        DeclareLaunchArgument('max_linear_speed_mps', default_value='0.45'),
        DeclareLaunchArgument('max_angular_speed_radps', default_value='0.70'),
        DeclareLaunchArgument('min_speed_percent', default_value='10'),
        DeclareLaunchArgument('max_speed_percent', default_value='30'),
        Node(
            package='golf_mower_bringup',
            executable='motor_driver_node.py',
            name='motor_driver',
            output='screen',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                'require_fresh_cmd_after_arm': ParameterValue(
                    LaunchConfiguration('require_fresh_cmd_after_arm'), value_type=bool),
                'command_rate_hz': ParameterValue(
                    LaunchConfiguration('command_rate_hz'), value_type=float),
                'cmd_vel_timeout_sec': ParameterValue(
                    LaunchConfiguration('cmd_vel_timeout_sec'), value_type=float),
                'stop_mode': LaunchConfiguration('stop_mode'),
                'max_linear_speed_mps': ParameterValue(
                    LaunchConfiguration('max_linear_speed_mps'), value_type=float),
                'max_angular_speed_radps': ParameterValue(
                    LaunchConfiguration('max_angular_speed_radps'), value_type=float),
                'min_speed_percent': ParameterValue(
                    LaunchConfiguration('min_speed_percent'), value_type=int),
                'max_speed_percent': ParameterValue(
                    LaunchConfiguration('max_speed_percent'), value_type=int),
            }],
        ),
    ])
