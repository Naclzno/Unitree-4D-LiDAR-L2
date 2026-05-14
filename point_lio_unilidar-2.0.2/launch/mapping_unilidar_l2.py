import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('point_lio_unilidar')
    config_file = os.path.join(pkg_dir, 'config', 'unilidar_l2_ros2.yaml')
    rviz_file = os.path.join(pkg_dir, 'rviz_cfg', 'loam_unilidar_display.rviz')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    config_arg = DeclareLaunchArgument('config_file', default_value=config_file)

    pointlio_node = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 1,
                'space_down_sample': True,
                'filter_size_surf': 0.4,
                'filter_size_map': 0.4,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        config_arg,
        pointlio_node,
        rviz_node,
    ])
