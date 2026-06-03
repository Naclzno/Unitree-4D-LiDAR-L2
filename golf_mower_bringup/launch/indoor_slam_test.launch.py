import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    point_lio_share = get_package_share_directory('point_lio_unilidar')

    point_lio_launch = os.path.join(
        point_lio_share, 'launch', 'mapping_unilidar_l2.py')
    default_pointlio_config = os.path.join(
        point_lio_share, 'config', 'unilidar_l2_ros2.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'indoor_slam.rviz')

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Start the Unitree L2 driver.'
    )
    use_pointlio_arg = DeclareLaunchArgument(
        'use_pointlio',
        default_value='true',
        description='Start Point-LIO for indoor SLAM testing.'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Start the bringup RViz with the Point-LIO display config.'
    )
    use_tf_adapter_arg = DeclareLaunchArgument(
        'use_tf_adapter',
        default_value='true',
        description='Publish temporary map/base frames for indoor bench testing.'
    )
    use_lidar_tf_adapter_arg = DeclareLaunchArgument(
        'use_lidar_tf_adapter',
        default_value='true',
        description='Attach Unitree lidar frames to base_link for bench testing.'
    )
    use_static_pointlio_pose_arg = DeclareLaunchArgument(
        'use_static_pointlio_pose',
        default_value='false',
        description='Publish a static camera_init -> aft_mapped transform for desk tests without Point-LIO odometry.'
    )
    map_to_camera_init_x_arg = DeclareLaunchArgument('map_to_camera_init_x', default_value='0.0')
    map_to_camera_init_y_arg = DeclareLaunchArgument('map_to_camera_init_y', default_value='0.0')
    map_to_camera_init_z_arg = DeclareLaunchArgument('map_to_camera_init_z', default_value='0.0')
    map_to_camera_init_roll_arg = DeclareLaunchArgument('map_to_camera_init_roll', default_value='0.0')
    map_to_camera_init_pitch_arg = DeclareLaunchArgument('map_to_camera_init_pitch', default_value='0.0')
    map_to_camera_init_yaw_arg = DeclareLaunchArgument('map_to_camera_init_yaw', default_value='0.0')
    initialize_type_arg = DeclareLaunchArgument(
        'initialize_type',
        default_value='2',
        description='Lidar initialization type: 1 for serial, 2 for UDP.'
    )
    work_mode_arg = DeclareLaunchArgument(
        'work_mode',
        default_value='0',
        description='Lidar work mode. Use 0 for UDP mode and 8 for serial mode.'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial device path used when initialize_type is 1.'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='4000000',
        description='Serial baudrate used when initialize_type is 1.'
    )
    start_lidar_rotation_arg = DeclareLaunchArgument(
        'start_lidar_rotation',
        default_value='true',
        description='Call startLidarRotation after initialization.'
    )
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument(
        'reset_lidar_after_set_mode',
        default_value='true',
        description='Call resetLidar after setting work mode.'
    )
    use_system_timestamp_arg = DeclareLaunchArgument(
        'use_system_timestamp',
        default_value='false',
        description='Use lidar hardware timestamps for Point-LIO cloud/IMU synchronization.'
    )
    pointlio_config_file_arg = DeclareLaunchArgument(
        'pointlio_config_file',
        default_value=default_pointlio_config,
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

    save_cloud_txt_arg = DeclareLaunchArgument(
        'save_cloud_txt',
        default_value='false',
        description='Save parsed Unitree point cloud frames as txt.'
    )
    cloud_txt_save_mode_arg = DeclareLaunchArgument(
        'cloud_txt_save_mode',
        default_value='overwrite_one_file',
        description='Point cloud txt save mode: overwrite_one_file or separate_files.'
    )
    cloud_txt_path_arg = DeclareLaunchArgument(
        'cloud_txt_path',
        default_value='/tmp/unitree_lidar_cloud.txt',
        description='Output txt path used by overwrite_one_file mode.'
    )
    cloud_txt_dir_arg = DeclareLaunchArgument(
        'cloud_txt_dir',
        default_value='/tmp/unitree_lidar_cloud_frames',
        description='Output directory used by separate_files mode.'
    )
    cloud_txt_save_every_n_arg = DeclareLaunchArgument(
        'cloud_txt_save_every_n',
        default_value='1',
        description='Save one point cloud frame every N parsed frames.'
    )

    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[
            {'initialize_type': ParameterValue(
                LaunchConfiguration('initialize_type'), value_type=int)},
            {'work_mode': ParameterValue(
                LaunchConfiguration('work_mode'), value_type=int)},
            {'use_system_timestamp': ParameterValue(
                LaunchConfiguration('use_system_timestamp'), value_type=bool)},
            {'start_lidar_rotation': ParameterValue(
                LaunchConfiguration('start_lidar_rotation'), value_type=bool)},
            {'reset_lidar_after_set_mode': ParameterValue(
                LaunchConfiguration('reset_lidar_after_set_mode'), value_type=bool)},
            {'publish_tf': False},
            {'imu_quaternion_order': LaunchConfiguration('imu_quaternion_order')},
            {'imu_angular_velocity_scale': ParameterValue(
                LaunchConfiguration('imu_angular_velocity_scale'), value_type=float)},
            {'imu_linear_acceleration_scale': ParameterValue(
                LaunchConfiguration('imu_linear_acceleration_scale'), value_type=float)},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 18},
            {'serial_port': LaunchConfiguration('serial_port')},
            {'baudrate': ParameterValue(
                LaunchConfiguration('baudrate'), value_type=int)},
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.1.62'},
            {'local_port': 6201},
            {'local_ip': '192.168.1.2'},
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
            {'save_cloud_txt': ParameterValue(
                LaunchConfiguration('save_cloud_txt'), value_type=bool)},
            {'cloud_txt_save_mode': LaunchConfiguration('cloud_txt_save_mode')},
            {'cloud_txt_path': LaunchConfiguration('cloud_txt_path')},
            {'cloud_txt_dir': LaunchConfiguration('cloud_txt_dir')},
            {'cloud_txt_save_every_n': ParameterValue(
                LaunchConfiguration('cloud_txt_save_every_n'), value_type=int)},
        ],
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(point_lio_launch),
        condition=IfCondition(LaunchConfiguration('use_pointlio')),
        launch_arguments={
            'use_rviz': 'false',
            'config_file': LaunchConfiguration('pointlio_config_file'),
        }.items(),
    )

    map_to_pointlio_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_tf_adapter')),
        arguments=[
            '--x', LaunchConfiguration('map_to_camera_init_x'),
            '--y', LaunchConfiguration('map_to_camera_init_y'),
            '--z', LaunchConfiguration('map_to_camera_init_z'),
            '--roll', LaunchConfiguration('map_to_camera_init_roll'),
            '--pitch', LaunchConfiguration('map_to_camera_init_pitch'),
            '--yaw', LaunchConfiguration('map_to_camera_init_yaw'),
            '--frame-id', 'map',
            '--child-frame-id', 'camera_init',
        ],
    )

    pointlio_body_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_mapped_to_base_link_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_tf_adapter')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'aft_mapped',
            '--child-frame-id', 'base_link',
        ],
    )

    static_pointlio_pose_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_aft_mapped_static_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_static_pointlio_pose')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_init',
            '--child-frame-id', 'aft_mapped',
        ],
    )

    static_pointlio_odom = Node(
        package='golf_mower_bringup',
        executable='static_pointlio_odom.py',
        name='static_pointlio_odom',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_static_pointlio_pose')),
        parameters=[{
            'odom_topic': '/pointlio/odom',
            'parent_frame': 'camera_init',
            'child_frame': 'aft_mapped',
            'publish_rate': 30.0,
        }],
    )

    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_unilidar_lidar_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar_tf_adapter')),
        arguments=[
            '--x', LaunchConfiguration('lidar_tf_x'),
            '--y', LaunchConfiguration('lidar_tf_y'),
            '--z', LaunchConfiguration('lidar_tf_z'),
            '--roll', LaunchConfiguration('lidar_tf_roll'),
            '--pitch', LaunchConfiguration('lidar_tf_pitch'),
            '--yaw', LaunchConfiguration('lidar_tf_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'unilidar_lidar',
        ],
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_unilidar_imu_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar_tf_adapter')),
        arguments=[
            '--x', LaunchConfiguration('imu_tf_x'),
            '--y', LaunchConfiguration('imu_tf_y'),
            '--z', LaunchConfiguration('imu_tf_z'),
            '--roll', LaunchConfiguration('imu_tf_roll'),
            '--pitch', LaunchConfiguration('imu_tf_pitch'),
            '--yaw', LaunchConfiguration('imu_tf_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'unilidar_imu',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_indoor_slam',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        launch_rviz_arg,
        use_tf_adapter_arg,
        use_lidar_tf_adapter_arg,
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
        save_cloud_txt_arg,
        cloud_txt_save_mode_arg,
        cloud_txt_path_arg,
        cloud_txt_dir_arg,
        cloud_txt_save_every_n_arg,
        lidar_node,
        pointlio,
        map_to_pointlio_map_tf,
        static_pointlio_pose_tf,
        static_pointlio_odom,
        pointlio_body_to_base_tf,
        base_to_lidar_tf,
        base_to_imu_tf,
        rviz_node,
    ])
