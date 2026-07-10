import os
from datetime import datetime

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
    patchwork_share = get_package_share_directory('patchworkpp')
    point_lio_share = get_package_share_directory('point_lio_unilidar')

    indoor_slam_launch = os.path.join(
        bringup_share, 'launch', 'indoor_slam_test.launch.py')
    patchwork_params = os.path.join(patchwork_share, 'config', 'params.yaml')
    default_pointlio_config = os.path.join(
        point_lio_share, 'config', 'unilidar_l2_ros2.yaml')
    map_session_name = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_map_session_dir = os.path.join(
        '/home/ubuntu/unilidar_sdk2/golf_mower_bringup/maps/stage5_segmented',
        map_session_name,
    )

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_patchwork_arg = DeclareLaunchArgument('use_patchwork', default_value='true')
    use_map_builder_arg = DeclareLaunchArgument('use_map_builder', default_value='true')
    use_map_metadata_recorder_arg = DeclareLaunchArgument(
        'use_map_metadata_recorder',
        default_value='false',
        description='Record RTK datum and Point-LIO start pose to map_metadata.yaml during offline mapping.'
    )
    use_um981_arg = DeclareLaunchArgument(
        'use_um981',
        default_value='false',
        description='Start UM981 ROS node and publish GNSS GGA as /fix. UM981 IMU/INS output is not used.'
    )
    use_fake_rtk_arg = DeclareLaunchArgument(
        'use_fake_rtk',
        default_value='false',
        description='Publish fake /fix from Point-LIO odometry so map metadata can be created indoors.'
    )
    use_pointlio_diagnostics_arg = DeclareLaunchArgument(
        'use_pointlio_diagnostics',
        default_value='false',
        description='Print Unitree cloud/IMU timing statistics and Point-LIO odom jump warnings.'
    )
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    use_static_pointlio_pose_arg = DeclareLaunchArgument(
        'use_static_pointlio_pose',
        default_value='false',
        description='Publish static camera_init -> aft_mapped for bench tests without Point-LIO odometry.'
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

    patchwork_cloud_topic_arg = DeclareLaunchArgument('patchwork_cloud_topic', default_value='/unilidar/cloud')
    patchwork_sensor_height_arg = DeclareLaunchArgument('patchwork_sensor_height', default_value='0.75')
    patchwork_min_r_arg = DeclareLaunchArgument('patchwork_min_r', default_value='0.2')
    patchwork_max_r_arg = DeclareLaunchArgument('patchwork_max_r', default_value='40.0')
    patchwork_log_every_n_arg = DeclareLaunchArgument('patchwork_log_every_n', default_value='60')

    segmented_map_output_dir_arg = DeclareLaunchArgument(
        'segmented_map_output_dir',
        default_value=default_map_session_dir,
        description='Per-run directory where ground_map.pcd and nonground_map.pcd are saved.'
    )
    segmented_map_voxel_resolution_arg = DeclareLaunchArgument(
        'segmented_map_voxel_resolution',
        default_value='0.05',
        description='Voxel resolution used while accumulating segmented PCD maps.'
    )
    segmented_map_save_period_arg = DeclareLaunchArgument(
        'segmented_map_save_period',
        default_value='10.0',
        description='Seconds between incremental segmented map saves.'
    )
    map_metadata_path_arg = DeclareLaunchArgument(
        'map_metadata_path',
        default_value=os.path.join(default_map_session_dir, 'map_metadata.yaml'),
        description='Path for offline map georeference metadata in the current run directory.'
    )
    fix_topic_arg = DeclareLaunchArgument('fix_topic', default_value='/fix')
    fake_rtk_datum_lat_arg = DeclareLaunchArgument(
        'fake_rtk_datum_lat', default_value='39.771981522833336')
    fake_rtk_datum_lon_arg = DeclareLaunchArgument(
        'fake_rtk_datum_lon', default_value='116.353032825')
    fake_rtk_datum_alt_arg = DeclareLaunchArgument(
        'fake_rtk_datum_alt', default_value='79.0')
    um981_port_arg = DeclareLaunchArgument(
        'um981_port',
        default_value='/dev/ttyUSB0',
        description='UM981 USB serial port. Prefer the /dev/serial/by-id/... path when it exists.'
    )
    um981_baud_arg = DeclareLaunchArgument('um981_baud', default_value='115200')
    um981_frame_id_arg = DeclareLaunchArgument('um981_frame_id', default_value='rtk_antenna')
    yaw_map_to_enu_arg = DeclareLaunchArgument(
        'yaw_map_to_enu',
        default_value='0.0',
        description='Yaw of offline map x-axis in ENU radians. Needed to convert RTK ENU into map coordinates.'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(indoor_slam_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_tf_adapter': 'true',
            'use_lidar_tf_adapter': 'true',
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

    map_builder = Node(
        package='golf_mower_bringup',
        executable='segmented_map_builder.py',
        name='segmented_map_builder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_map_builder')),
        parameters=[{
            'ground_topic': '/ground_segmentation/ground',
            'nonground_topic': '/ground_segmentation/nonground',
            'target_frame': 'map',
            'output_dir': LaunchConfiguration('segmented_map_output_dir'),
            'voxel_resolution': ParameterValue(LaunchConfiguration('segmented_map_voxel_resolution'), value_type=float),
            'save_period_sec': ParameterValue(LaunchConfiguration('segmented_map_save_period'), value_type=float),
        }],
    )

    pointlio_diagnostics = Node(
        package='golf_mower_bringup',
        executable='pointlio_input_diagnostics.py',
        name='pointlio_input_diagnostics',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_pointlio_diagnostics')),
        parameters=[{
            'cloud_topic': '/unilidar/cloud',
            'imu_topic': '/unilidar/imu',
            'odom_topic': '/pointlio/odom',
            'cloud_print_every_n': 10,
            'imu_window_size': 300,
            'odom_jump_distance': 0.1,
            'odom_jump_speed': 1.0,
        }],
    )

    map_metadata_recorder = Node(
        package='golf_mower_bringup',
        executable='map_metadata_recorder.py',
        name='map_metadata_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_map_metadata_recorder')),
        parameters=[{
            'fix_topic': LaunchConfiguration('fix_topic'),
            'odom_topic': '/pointlio/odom',
            'output_path': LaunchConfiguration('map_metadata_path'),
            'map_frame': 'map',
            'pointlio_map_frame': 'camera_init',
            'pointlio_body_frame': 'aft_mapped',
            'yaw_map_to_enu': ParameterValue(LaunchConfiguration('yaw_map_to_enu'), value_type=float),
            'overwrite': False,
        }],
    )

    um981_node = Node(
        package='um981_ros',
        executable='um981_node',
        name='um981_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_um981')),
        remappings=[
            ('/fix', LaunchConfiguration('fix_topic')),
        ],
        parameters=[{
            'port': LaunchConfiguration('um981_port'),
            'baud': ParameterValue(LaunchConfiguration('um981_baud'), value_type=int),
            'frame_id': LaunchConfiguration('um981_frame_id'),
            'imu_frame_id': 'um981_imu',
            'commands': ['GNGGA 1'],
        }],
    )

    fake_rtk = Node(
        package='golf_mower_bringup',
        executable='fake_rtk_from_odom.py',
        name='fake_rtk_from_odom',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_rtk')),
        parameters=[{
            'odom_topic': '/pointlio/odom',
            'fix_topic': LaunchConfiguration('fix_topic'),
            'frame_id': LaunchConfiguration('um981_frame_id'),
            'datum_lat_deg': ParameterValue(LaunchConfiguration('fake_rtk_datum_lat'), value_type=float),
            'datum_lon_deg': ParameterValue(LaunchConfiguration('fake_rtk_datum_lon'), value_type=float),
            'datum_alt_m': ParameterValue(LaunchConfiguration('fake_rtk_datum_alt'), value_type=float),
        }],
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_patchwork_arg,
        use_map_builder_arg,
        use_map_metadata_recorder_arg,
        use_um981_arg,
        use_fake_rtk_arg,
        use_pointlio_diagnostics_arg,
        launch_rviz_arg,
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
        patchwork_cloud_topic_arg,
        patchwork_sensor_height_arg,
        patchwork_min_r_arg,
        patchwork_max_r_arg,
        patchwork_log_every_n_arg,
        segmented_map_output_dir_arg,
        segmented_map_voxel_resolution_arg,
        segmented_map_save_period_arg,
        map_metadata_path_arg,
        fix_topic_arg,
        fake_rtk_datum_lat_arg,
        fake_rtk_datum_lon_arg,
        fake_rtk_datum_alt_arg,
        um981_port_arg,
        um981_baud_arg,
        um981_frame_id_arg,
        yaw_map_to_enu_arg,
        slam,
        patchwork,
        map_builder,
        pointlio_diagnostics,
        map_metadata_recorder,
        um981_node,
        fake_rtk,
    ])
