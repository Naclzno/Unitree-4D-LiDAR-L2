import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    stage2_launch = os.path.join(
        bringup_share, 'launch', 'outdoor_elevation_stage2.launch.py')
    nav2_launch = os.path.join(
        nav2_share, 'launch', 'navigation_launch.py')
    nav2_params = os.path.join(
        bringup_share, 'config', 'nav2_outdoor_mock.yaml')

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_elevation_arg = DeclareLaunchArgument('use_elevation', default_value='true')
    use_grid_converter_arg = DeclareLaunchArgument('use_grid_converter', default_value='true')
    use_nav2_arg = DeclareLaunchArgument('use_nav2', default_value='true')
    launch_outdoor_rviz_arg = DeclareLaunchArgument('launch_outdoor_rviz', default_value='true')
    use_lidar_tf_adapter_arg = DeclareLaunchArgument('use_lidar_tf_adapter', default_value='true')
    initialize_type_arg = DeclareLaunchArgument('initialize_type', default_value='2')
    work_mode_arg = DeclareLaunchArgument('work_mode', default_value='0')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='4000000')
    start_lidar_rotation_arg = DeclareLaunchArgument('start_lidar_rotation', default_value='true')
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument('reset_lidar_after_set_mode', default_value='true')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Nav2 parameter file for outdoor elevation mapping integration.'
    )
    nav2_start_delay_arg = DeclareLaunchArgument(
        'nav2_start_delay',
        default_value='25.0',
        description='Seconds to wait before starting Nav2 so elevation/traversability topics are available.'
    )

    stage2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage2_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'use_elevation': LaunchConfiguration('use_elevation'),
            'use_grid_converter': LaunchConfiguration('use_grid_converter'),
            'launch_outdoor_rviz': LaunchConfiguration('launch_outdoor_rviz'),
            'use_lidar_tf_adapter': LaunchConfiguration('use_lidar_tf_adapter'),
            'initialize_type': LaunchConfiguration('initialize_type'),
            'work_mode': LaunchConfiguration('work_mode'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'start_lidar_rotation': LaunchConfiguration('start_lidar_rotation'),
            'reset_lidar_after_set_mode': LaunchConfiguration('reset_lidar_after_set_mode'),
        }.items(),
    )

    nav2 = TimerAction(
        period=LaunchConfiguration('nav2_start_delay'),
        actions=[
            IncludeLaunchDescription(
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
        ],
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_elevation_arg,
        use_grid_converter_arg,
        use_nav2_arg,
        launch_outdoor_rviz_arg,
        use_lidar_tf_adapter_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        params_file_arg,
        nav2_start_delay_arg,
        stage2,
        nav2,
    ])
