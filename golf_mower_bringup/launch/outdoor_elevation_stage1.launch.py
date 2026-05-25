import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    elevation_share = get_package_share_directory('elevation_mapping_cupy')

    indoor_slam_launch = os.path.join(
        bringup_share, 'launch', 'indoor_slam_test.launch.py')
    outdoor_rviz_config = os.path.join(
        bringup_share, 'rviz', 'outdoor_elevation.rviz')
    elevation_core_config = os.path.join(
        elevation_share, 'config', 'core', 'core_param.yaml')
    elevation_unitree_config = os.path.join(
        bringup_share, 'config', 'elevation_unitree_l2_stage1.yaml')

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_elevation_arg = DeclareLaunchArgument('use_elevation', default_value='true')
    launch_outdoor_rviz_arg = DeclareLaunchArgument('launch_outdoor_rviz', default_value='true')
    use_lidar_tf_adapter_arg = DeclareLaunchArgument('use_lidar_tf_adapter', default_value='true')
    initialize_type_arg = DeclareLaunchArgument('initialize_type', default_value='2')
    work_mode_arg = DeclareLaunchArgument('work_mode', default_value='0')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='4000000')
    start_lidar_rotation_arg = DeclareLaunchArgument('start_lidar_rotation', default_value='true')
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument('reset_lidar_after_set_mode', default_value='true')

    indoor_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(indoor_slam_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'launch_rviz': 'false',
            'use_tf_adapter': 'true',
            'use_lidar_tf_adapter': LaunchConfiguration('use_lidar_tf_adapter'),
            'initialize_type': LaunchConfiguration('initialize_type'),
            'work_mode': LaunchConfiguration('work_mode'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'start_lidar_rotation': LaunchConfiguration('start_lidar_rotation'),
            'reset_lidar_after_set_mode': LaunchConfiguration('reset_lidar_after_set_mode'),
        }.items(),
    )

    elevation_mapping_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_elevation')),
        parameters=[
            elevation_core_config,
            elevation_unitree_config,
            {'use_sim_time': False},
        ],
    )
    elevation_mapping = TimerAction(
        period=15.0,
        actions=[elevation_mapping_node],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_outdoor_elevation',
        arguments=['-d', outdoor_rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_outdoor_rviz')),
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_elevation_arg,
        launch_outdoor_rviz_arg,
        use_lidar_tf_adapter_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        indoor_slam,
        elevation_mapping,
        rviz_node,
    ])
