import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')

    indoor_slam_launch = os.path.join(
        bringup_share, 'launch', 'indoor_slam_test.launch.py')
    default_outdoor_rviz_config = os.path.join(
        bringup_share, 'rviz', 'outdoor_elevation.rviz')
    elevation_ros2_core_config = os.path.join(
        bringup_share, 'config', 'elevation_core_common.yaml')
    default_elevation_unitree_config = os.path.join(
        bringup_share, 'config', 'elevation_unitree_l2_stage1.yaml')
    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_elevation_arg = DeclareLaunchArgument('use_elevation', default_value='true')
    use_cuda_elevation_arg = DeclareLaunchArgument(
        'use_cuda_elevation',
        default_value='false',
        description='Use elevation_mapping_cupy when true; use CPU-only elevation_mapping_ros2 when false.'
    )
    launch_outdoor_rviz_arg = DeclareLaunchArgument('launch_outdoor_rviz', default_value='true')
    use_lidar_tf_adapter_arg = DeclareLaunchArgument('use_lidar_tf_adapter', default_value='true')
    use_map_to_camera_init_adapter_arg = DeclareLaunchArgument('use_map_to_camera_init_adapter', default_value='true')
    use_static_pointlio_pose_arg = DeclareLaunchArgument(
        'use_static_pointlio_pose',
        default_value='false',
        description='Publish static camera_init -> aft_mapped for desk tests without Point-LIO odometry.'
    )
    map_to_camera_init_x_arg = DeclareLaunchArgument('map_to_camera_init_x', default_value='0.0')
    map_to_camera_init_y_arg = DeclareLaunchArgument('map_to_camera_init_y', default_value='0.0')
    map_to_camera_init_z_arg = DeclareLaunchArgument('map_to_camera_init_z', default_value='0.0')
    map_to_camera_init_roll_arg = DeclareLaunchArgument('map_to_camera_init_roll', default_value='0.0')
    map_to_camera_init_pitch_arg = DeclareLaunchArgument('map_to_camera_init_pitch', default_value='0.0')
    map_to_camera_init_yaw_arg = DeclareLaunchArgument('map_to_camera_init_yaw', default_value='0.0')
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
    lidar_tf_x_arg = DeclareLaunchArgument('lidar_tf_x', default_value='0.0')
    lidar_tf_y_arg = DeclareLaunchArgument('lidar_tf_y', default_value='0.0')
    lidar_tf_z_arg = DeclareLaunchArgument('lidar_tf_z', default_value='0.0')
    lidar_tf_roll_arg = DeclareLaunchArgument('lidar_tf_roll', default_value='0.0')
    lidar_tf_pitch_arg = DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0')
    lidar_tf_yaw_arg = DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0')
    imu_tf_x_arg = DeclareLaunchArgument('imu_tf_x', default_value='-0.007698')
    imu_tf_y_arg = DeclareLaunchArgument('imu_tf_y', default_value='-0.014655')
    imu_tf_z_arg = DeclareLaunchArgument('imu_tf_z', default_value='0.00667')
    imu_tf_roll_arg = DeclareLaunchArgument('imu_tf_roll', default_value='0.0')
    imu_tf_pitch_arg = DeclareLaunchArgument('imu_tf_pitch', default_value='0.0')
    imu_tf_yaw_arg = DeclareLaunchArgument('imu_tf_yaw', default_value='0.0')
    elevation_unitree_config_arg = DeclareLaunchArgument(
        'elevation_unitree_config',
        default_value=default_elevation_unitree_config,
        description='Unitree L2 specific elevation_mapping_cupy parameter file.'
    )
    outdoor_rviz_config_arg = DeclareLaunchArgument(
        'outdoor_rviz_config',
        default_value=default_outdoor_rviz_config,
        description='RViz config used by the outdoor elevation launch.'
    )

    indoor_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(indoor_slam_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'launch_rviz': 'false',
            'use_tf_adapter': 'true',
            'use_map_to_camera_init_adapter': LaunchConfiguration('use_map_to_camera_init_adapter'),
            'use_lidar_tf_adapter': LaunchConfiguration('use_lidar_tf_adapter'),
            'use_static_pointlio_pose': LaunchConfiguration('use_static_pointlio_pose'),
            'map_to_camera_init_x': LaunchConfiguration('map_to_camera_init_x'),
            'map_to_camera_init_y': LaunchConfiguration('map_to_camera_init_y'),
            'map_to_camera_init_z': LaunchConfiguration('map_to_camera_init_z'),
            'map_to_camera_init_roll': LaunchConfiguration('map_to_camera_init_roll'),
            'map_to_camera_init_pitch': LaunchConfiguration('map_to_camera_init_pitch'),
            'map_to_camera_init_yaw': LaunchConfiguration('map_to_camera_init_yaw'),
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
            'lidar_tf_x': LaunchConfiguration('lidar_tf_x'),
            'lidar_tf_y': LaunchConfiguration('lidar_tf_y'),
            'lidar_tf_z': LaunchConfiguration('lidar_tf_z'),
            'lidar_tf_roll': LaunchConfiguration('lidar_tf_roll'),
            'lidar_tf_pitch': LaunchConfiguration('lidar_tf_pitch'),
            'lidar_tf_yaw': LaunchConfiguration('lidar_tf_yaw'),
            'imu_tf_x': LaunchConfiguration('imu_tf_x'),
            'imu_tf_y': LaunchConfiguration('imu_tf_y'),
            'imu_tf_z': LaunchConfiguration('imu_tf_z'),
            'imu_tf_roll': LaunchConfiguration('imu_tf_roll'),
            'imu_tf_pitch': LaunchConfiguration('imu_tf_pitch'),
            'imu_tf_yaw': LaunchConfiguration('imu_tf_yaw'),
        }.items(),
    )

    def _launch_elevation_node(context, *args, **kwargs):
        use_cuda = LaunchConfiguration('use_cuda_elevation').perform(context).lower() in (
            'true', '1', 'yes'
        )
        if use_cuda:
            package = 'elevation_mapping_cupy'
            core_config = os.path.join(
                get_package_share_directory('elevation_mapping_cupy'),
                'config', 'core', 'core_param.yaml')
        else:
            package = 'elevation_mapping_ros2'
            core_config = elevation_ros2_core_config
        return [
            Node(
                package=package,
                executable='elevation_mapping_node.py',
                name='elevation_mapping_node',
                output='screen',
                parameters=[
                    core_config,
                    LaunchConfiguration('elevation_unitree_config'),
                    {'use_sim_time': False},
                ],
            )
        ]

    elevation_mapping = TimerAction(
        period=15.0,
        condition=IfCondition(LaunchConfiguration('use_elevation')),
        actions=[OpaqueFunction(function=_launch_elevation_node)],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_outdoor_elevation',
        arguments=['-d', LaunchConfiguration('outdoor_rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_outdoor_rviz')),
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_elevation_arg,
        use_cuda_elevation_arg,
        launch_outdoor_rviz_arg,
        use_lidar_tf_adapter_arg,
        use_map_to_camera_init_adapter_arg,
        use_static_pointlio_pose_arg,
        map_to_camera_init_x_arg,
        map_to_camera_init_y_arg,
        map_to_camera_init_z_arg,
        map_to_camera_init_roll_arg,
        map_to_camera_init_pitch_arg,
        map_to_camera_init_yaw_arg,
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
        lidar_tf_x_arg,
        lidar_tf_y_arg,
        lidar_tf_z_arg,
        lidar_tf_roll_arg,
        lidar_tf_pitch_arg,
        lidar_tf_yaw_arg,
        imu_tf_x_arg,
        imu_tf_y_arg,
        imu_tf_z_arg,
        imu_tf_roll_arg,
        imu_tf_pitch_arg,
        imu_tf_yaw_arg,
        elevation_unitree_config_arg,
        outdoor_rviz_config_arg,
        indoor_slam,
        elevation_mapping,
        rviz_node,
    ])
