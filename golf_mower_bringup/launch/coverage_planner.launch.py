import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('golf_mower_bringup')
    default_area = os.path.join(share, 'config', 'coverage_test_area.yaml')
    default_rviz = os.path.join(share, 'rviz', 'coverage_planner.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('area_file', default_value=default_area),
        DeclareLaunchArgument(
            'output_file',
            default_value='~/.ros/golf_mower/coverage_path.yaml'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('path_pose_spacing', default_value='0.10'),
        DeclareLaunchArgument('nav_waypoint_spacing', default_value='0.75'),
        DeclareLaunchArgument('republish_period_sec', default_value='2.0'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        Node(
            package='golf_mower_bringup',
            executable='coverage_planner_node',
            name='coverage_planner',
            output='screen',
            parameters=[{
                'area_file': LaunchConfiguration('area_file'),
                'output_file': LaunchConfiguration('output_file'),
                'dry_run': ParameterValue(
                    LaunchConfiguration('dry_run'), value_type=bool),
                'path_pose_spacing': ParameterValue(
                    LaunchConfiguration('path_pose_spacing'), value_type=float),
                'nav_waypoint_spacing': ParameterValue(
                    LaunchConfiguration('nav_waypoint_spacing'), value_type=float),
                'republish_period_sec': ParameterValue(
                    LaunchConfiguration('republish_period_sec'), value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='coverage_planner_rviz',
            arguments=['-d', default_rviz],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
