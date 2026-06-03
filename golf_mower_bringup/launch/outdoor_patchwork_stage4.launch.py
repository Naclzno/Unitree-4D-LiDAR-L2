import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    patchwork_share = get_package_share_directory('patchworkpp')
    nav2_share = get_package_share_directory('nav2_bringup')

    stage2_launch = os.path.join(
        bringup_share, 'launch', 'outdoor_elevation_stage2.launch.py')
    nav2_launch = os.path.join(nav2_share, 'launch', 'navigation_launch.py')
    patchwork_params = os.path.join(patchwork_share, 'config', 'params.yaml')
    elevation_patchwork_config = os.path.join(
        bringup_share, 'config', 'elevation_unitree_l2_patchwork_stage4.yaml')
    nav2_patchwork_params = os.path.join(
        bringup_share, 'config', 'nav2_outdoor_patchwork.yaml')
    outdoor_rviz_config = os.path.join(
        bringup_share, 'rviz', 'outdoor_elevation.rviz')

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_patchwork_arg = DeclareLaunchArgument('use_patchwork', default_value='true')
    use_ground_fallback_arg = DeclareLaunchArgument('use_ground_fallback', default_value='true')
    use_elevation_arg = DeclareLaunchArgument('use_elevation', default_value='true')
    use_grid_converter_arg = DeclareLaunchArgument('use_grid_converter', default_value='true')
    use_nav2_arg = DeclareLaunchArgument('use_nav2', default_value='true')
    launch_outdoor_rviz_arg = DeclareLaunchArgument('launch_outdoor_rviz', default_value='true')
    use_lidar_tf_adapter_arg = DeclareLaunchArgument('use_lidar_tf_adapter', default_value='true')
    use_static_pointlio_pose_arg = DeclareLaunchArgument(
        'use_static_pointlio_pose',
        default_value='false',
        description='Publish static camera_init -> aft_mapped for desk tests without Point-LIO odometry.'
    )

    initialize_type_arg = DeclareLaunchArgument('initialize_type', default_value='2')
    work_mode_arg = DeclareLaunchArgument('work_mode', default_value='0')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='4000000')
    start_lidar_rotation_arg = DeclareLaunchArgument('start_lidar_rotation', default_value='true')
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument('reset_lidar_after_set_mode', default_value='true')
    use_system_timestamp_arg = DeclareLaunchArgument('use_system_timestamp', default_value='false')
    pointlio_config_file_arg = DeclareLaunchArgument(
        'pointlio_config_file',
        default_value=os.path.join(
            get_package_share_directory('point_lio_unilidar'),
            'config',
            'unilidar_l2_ros2.yaml'),
        description='Point-LIO config file.'
    )
    imu_quaternion_order_arg = DeclareLaunchArgument(
        'imu_quaternion_order',
        default_value='wxyz',
        description='Order of Unitree SDK quaternion values: wxyz or xyzw.'
    )
    imu_linear_acceleration_scale_arg = DeclareLaunchArgument(
        'imu_linear_acceleration_scale',
        default_value='1.0',
        description='Scale Unitree SDK IMU acceleration before publishing /unilidar/imu in m/s^2.'
    )
    imu_angular_velocity_scale_arg = DeclareLaunchArgument(
        'imu_angular_velocity_scale',
        default_value='0.017453292519943295',
        description='Scale Unitree SDK IMU angular velocity before publishing /unilidar/imu in rad/s.'
    )

    patchwork_cloud_topic_arg = DeclareLaunchArgument(
        'patchwork_cloud_topic',
        default_value='/unilidar/cloud',
        description='Input PointCloud2 topic for Patchwork++ ground segmentation.'
    )
    patchwork_sensor_height_arg = DeclareLaunchArgument(
        'patchwork_sensor_height',
        default_value='0.80',
        description='Approximate lidar mounting height in meters for Patchwork++.'
    )
    patchwork_min_r_arg = DeclareLaunchArgument(
        'patchwork_min_r',
        default_value='0.2',
        description='Minimum range used by Patchwork++.'
    )
    patchwork_max_r_arg = DeclareLaunchArgument(
        'patchwork_max_r',
        default_value='40.0',
        description='Maximum range used by Patchwork++.'
    )
    patchwork_log_every_n_arg = DeclareLaunchArgument(
        'patchwork_log_every_n',
        default_value='60',
        description='Patchwork++ prints one segmentation summary every N frames; 0 disables the summary.'
    )
    min_ground_points_arg = DeclareLaunchArgument(
        'min_ground_points',
        default_value='100',
        description='Minimum Patchwork++ ground points required before elevation mapping uses the segmented ground cloud.'
    )
    nav2_start_delay_arg = DeclareLaunchArgument(
        'nav2_start_delay',
        default_value='25.0',
        description='Seconds to wait before starting Nav2 so segmentation and elevation topics are available.'
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
            'use_static_pointlio_pose': LaunchConfiguration('use_static_pointlio_pose'),
            'initialize_type': LaunchConfiguration('initialize_type'),
            'work_mode': LaunchConfiguration('work_mode'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'start_lidar_rotation': LaunchConfiguration('start_lidar_rotation'),
            'reset_lidar_after_set_mode': LaunchConfiguration('reset_lidar_after_set_mode'),
            'use_system_timestamp': LaunchConfiguration('use_system_timestamp'),
            'pointlio_config_file': LaunchConfiguration('pointlio_config_file'),
            'imu_quaternion_order': LaunchConfiguration('imu_quaternion_order'),
            'imu_angular_velocity_scale': LaunchConfiguration('imu_angular_velocity_scale'),
            'imu_linear_acceleration_scale': LaunchConfiguration('imu_linear_acceleration_scale'),
            'elevation_unitree_config': elevation_patchwork_config,
            'outdoor_rviz_config': outdoor_rviz_config,
        }.items(),
    )

    patchwork = Node(
        package='patchworkpp',
        executable='demo',
        name='ground_segmentation',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_patchwork')),
        parameters=[
            patchwork_params,
            {
                'cloud_topic': LaunchConfiguration('patchwork_cloud_topic'),
                'sensor_height': ParameterValue(LaunchConfiguration('patchwork_sensor_height'), value_type=float),
                'min_r': ParameterValue(LaunchConfiguration('patchwork_min_r'), value_type=float),
                'max_r': ParameterValue(LaunchConfiguration('patchwork_max_r'), value_type=float),
                'log_every_n': ParameterValue(LaunchConfiguration('patchwork_log_every_n'), value_type=int),
                'visualize': False,
            },
        ],
    )

    ground_fallback = Node(
        package='golf_mower_bringup',
        executable='pointcloud_ground_fallback.py',
        name='pointcloud_ground_fallback',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ground_fallback')),
        parameters=[{
            'raw_cloud_topic': '/unilidar/cloud',
            'ground_cloud_topic': '/ground_segmentation/ground',
            'output_cloud_topic': '/golf_mower/ground_cloud_for_elevation',
            'min_ground_points': ParameterValue(LaunchConfiguration('min_ground_points'), value_type=int),
        }],
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
                    'params_file': nav2_patchwork_params,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_patchwork_arg,
        use_ground_fallback_arg,
        use_elevation_arg,
        use_grid_converter_arg,
        use_nav2_arg,
        launch_outdoor_rviz_arg,
        use_lidar_tf_adapter_arg,
        use_static_pointlio_pose_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        use_system_timestamp_arg,
        pointlio_config_file_arg,
        imu_quaternion_order_arg,
        imu_angular_velocity_scale_arg,
        imu_linear_acceleration_scale_arg,
        patchwork_cloud_topic_arg,
        patchwork_sensor_height_arg,
        patchwork_min_r_arg,
        patchwork_max_r_arg,
        patchwork_log_every_n_arg,
        min_ground_points_arg,
        nav2_start_delay_arg,
        stage2,
        patchwork,
        ground_fallback,
        nav2,
    ])
