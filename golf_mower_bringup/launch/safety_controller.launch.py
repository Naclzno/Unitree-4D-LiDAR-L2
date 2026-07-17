from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('output_cmd_vel_topic', default_value='/motor/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/pointlio/odom'),
        DeclareLaunchArgument('emergency_stop_topic', default_value='/emergency_stop'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('cmd_vel_timeout_sec', default_value='0.50'),
        DeclareLaunchArgument('odom_timeout_sec', default_value='0.50'),
        DeclareLaunchArgument('require_odom', default_value='true'),
        Node(
            package='golf_mower_bringup',
            executable='safety_controller_node.py',
            name='safety_controller',
            output='screen',
            parameters=[{
                'input_cmd_vel_topic': LaunchConfiguration('input_cmd_vel_topic'),
                'output_cmd_vel_topic': LaunchConfiguration('output_cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'emergency_stop_topic': LaunchConfiguration('emergency_stop_topic'),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'), value_type=float),
                'cmd_vel_timeout_sec': ParameterValue(
                    LaunchConfiguration('cmd_vel_timeout_sec'), value_type=float),
                'odom_timeout_sec': ParameterValue(
                    LaunchConfiguration('odom_timeout_sec'), value_type=float),
                'require_odom': ParameterValue(
                    LaunchConfiguration('require_odom'), value_type=bool),
            }],
        ),
    ])
