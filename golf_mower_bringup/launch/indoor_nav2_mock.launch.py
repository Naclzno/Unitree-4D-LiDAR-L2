import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    indoor_slam_launch = os.path.join(
        bringup_share, 'launch', 'indoor_slam_test.launch.py')
    nav2_launch = os.path.join(
        nav2_share, 'launch', 'navigation_launch.py')
    nav2_params = os.path.join(
        bringup_share, 'config', 'nav2_indoor_mock.yaml')

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    use_nav2_arg = DeclareLaunchArgument('use_nav2', default_value='true')
    use_lidar_tf_adapter_arg = DeclareLaunchArgument('use_lidar_tf_adapter', default_value='true')
    initialize_type_arg = DeclareLaunchArgument('initialize_type', default_value='2')
    work_mode_arg = DeclareLaunchArgument('work_mode', default_value='0')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='4000000')
    start_lidar_rotation_arg = DeclareLaunchArgument('start_lidar_rotation', default_value='true')
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument('reset_lidar_after_set_mode', default_value='true')
    use_odom_tf_bridge_arg = DeclareLaunchArgument(
        'use_odom_tf_bridge',
        default_value='true',
        description='Publish camera_init -> aft_mapped TF from /pointlio/odom.'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Nav2 parameter file for indoor Point-LIO mock integration.'
    )

    indoor_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(indoor_slam_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
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

    odom_tf_bridge = Node(
        package='golf_mower_bringup',
        executable='pointlio_odom_tf_bridge.py',
        name='pointlio_odom_tf_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_odom_tf_bridge')),
        parameters=[{
            'odom_topic': '/pointlio/odom',
            'parent_frame': 'camera_init',
            'child_frame': 'aft_mapped',
            'use_odom_frame_ids': True,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        launch_rviz_arg,
        use_nav2_arg,
        use_lidar_tf_adapter_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        use_odom_tf_bridge_arg,
        params_file_arg,
        indoor_slam,
        odom_tf_bridge,
        nav2,
    ])
