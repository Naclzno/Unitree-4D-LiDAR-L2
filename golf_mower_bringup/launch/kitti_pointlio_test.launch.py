import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    pointlio_params = os.path.join(
        bringup_share, 'config', 'pointlio_kitti_ros2.yaml')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/ubuntu/unilidar_sdk2/point_lio_unilidar-2.0.2/kitti_00_sample_ros2',
        description='Path to the KITTI ROS2 bag directory.'
    )
    use_bag_arg = DeclareLaunchArgument('use_bag', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')

    bag_play = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_bag')),
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
        ],
        output='screen',
    )

    pointlio_node = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_pointlio')),
        parameters=[
            pointlio_params,
            {
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 4,
                'space_down_sample': True,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
            },
        ],
    )

    map_to_pointlio_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'camera_init',
        ],
    )

    pointlio_body_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_mapped_to_base_link_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'aft_mapped',
            '--child-frame-id', 'base_link',
        ],
    )

    odom_tf_bridge = Node(
        package='golf_mower_bringup',
        executable='pointlio_odom_tf_bridge.py',
        name='pointlio_odom_tf_bridge',
        output='screen',
        parameters=[{
            'odom_topic': '/pointlio/odom',
            'parent_frame': 'camera_init',
            'child_frame': 'aft_mapped',
            'use_odom_frame_ids': True,
        }],
    )

    return LaunchDescription([
        bag_path_arg,
        use_bag_arg,
        use_pointlio_arg,
        bag_play,
        pointlio_node,
        map_to_pointlio_map_tf,
        pointlio_body_to_base_tf,
        odom_tf_bridge,
    ])
