import os
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


STAGE5_MAP_ROOT = '/home/ubuntu/unilidar_sdk2/golf_mower_bringup/maps/stage5_segmented'


def _latest_complete_map_dir(root_dir):
    if os.path.isdir(root_dir):
        for name in sorted(os.listdir(root_dir), reverse=True):
            candidate = os.path.join(root_dir, name)
            required = ('ground_map.pcd', 'nonground_map.pcd', 'map_metadata.yaml')
            if os.path.isdir(candidate) and all(
                os.path.isfile(os.path.join(candidate, filename)) for filename in required
            ):
                return candidate
    return root_dir


def _pcd_point_count(path):
    if not os.path.isfile(path):
        raise RuntimeError(f'Stage 5 navigation aborted: ground map does not exist: {path}')

    header = {}
    with open(path, 'rb') as pcd:
        for _ in range(128):
            raw_line = pcd.readline()
            if not raw_line:
                break
            try:
                line = raw_line.decode('ascii').strip()
            except UnicodeDecodeError as exc:
                raise RuntimeError(
                    f'Stage 5 navigation aborted: invalid PCD header in {path}'
                ) from exc
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == 'DATA':
                break

    try:
        if header.get('POINTS'):
            return int(header['POINTS'][0])
        if header.get('WIDTH') and header.get('HEIGHT'):
            return int(header['WIDTH'][0]) * int(header['HEIGHT'][0])
    except (ValueError, IndexError) as exc:
        raise RuntimeError(
            f'Stage 5 navigation aborted: invalid point count in PCD header: {path}'
        ) from exc
    raise RuntimeError(f'Stage 5 navigation aborted: PCD point count is missing: {path}')


def _validate_ground_map(context):
    map_dir = LaunchConfiguration('segmented_map_dir').perform(context)
    ground_path = os.path.join(os.path.expanduser(map_dir), 'ground_map.pcd')
    point_count = _pcd_point_count(ground_path)
    if point_count <= 0:
        allow_recovery = LaunchConfiguration('allow_offline_ground_recovery').perform(context).lower()
        if allow_recovery not in ('1', 'true', 'yes', 'on'):
            raise RuntimeError(
                'Stage 5 navigation aborted: ground_map.pcd contains 0 points and '
                f'offline recovery is disabled. File: {ground_path}')
        helper_path = os.path.join(os.path.dirname(__file__), 'offline_ground_recovery.py')
        spec = importlib.util.spec_from_file_location('offline_ground_recovery', helper_path)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        metadata_path = LaunchConfiguration('map_metadata_path').perform(context)
        sensor_height = float(LaunchConfiguration('patchwork_sensor_height').perform(context))
        recovered_ground, recovered_nonground, report = helper.recover_ground_map(
            os.path.expanduser(map_dir),
            os.path.expanduser(metadata_path),
            sensor_height=sensor_height,
        )
        print(
            '[stage5_map_check] offline ground recovered: '
            f'ground_points={report["result"]["ground_points"]}, '
            f'ground_cells={report["result"]["connected_ground_cells"]}, '
            f'file={recovered_ground}')
        return [
            SetLaunchConfiguration('ground_map_filename', os.path.basename(recovered_ground)),
            SetLaunchConfiguration('nonground_map_filename', os.path.basename(recovered_nonground)),
        ]
    print(f'[stage5_map_check] ground_map.pcd points={point_count}, file={ground_path}')
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory('golf_mower_bringup')
    patchwork_share = get_package_share_directory('patchworkpp')
    nav2_share = get_package_share_directory('nav2_bringup')

    stage2_launch = os.path.join(
        bringup_share, 'launch', 'outdoor_elevation_stage2.launch.py')
    nav2_launch = os.path.join(nav2_share, 'launch', 'navigation_launch.py')
    nav2_params = os.path.join(
        bringup_share, 'config', 'nav2_outdoor_stage5_segmented.yaml')
    elevation_config = os.path.join(
        bringup_share, 'config', 'elevation_unitree_l2_patchwork_stage4.yaml')
    patchwork_params = os.path.join(patchwork_share, 'config', 'params.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'outdoor_elevation.rviz')
    default_map_session_dir = _latest_complete_map_dir(STAGE5_MAP_ROOT)

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_elevation_arg = DeclareLaunchArgument('use_elevation', default_value='true')
    use_cuda_elevation_arg = DeclareLaunchArgument(
        'use_cuda_elevation',
        default_value='true',
        description='Use elevation_mapping_cupy when true; use CPU-only elevation_mapping_ros2 when false.'
    )
    use_grid_converter_arg = DeclareLaunchArgument('use_grid_converter', default_value='true')
    use_patchwork_arg = DeclareLaunchArgument('use_patchwork', default_value='true')
    use_ground_fallback_arg = DeclareLaunchArgument('use_ground_fallback', default_value='true')
    allow_offline_ground_recovery_arg = DeclareLaunchArgument(
        'allow_offline_ground_recovery',
        default_value='true',
        description='Recover a connected low ground surface from nonground_map.pcd when ground_map.pcd is empty.'
    )
    use_rtk_map_localizer_arg = DeclareLaunchArgument(
        'use_rtk_map_localizer',
        default_value='false',
        description='Use /fix and map_metadata.yaml to publish map -> camera_init for offline map localization.'
    )
    use_fake_rtk_arg = DeclareLaunchArgument(
        'use_fake_rtk',
        default_value='false',
        description='Publish fake /fix from /pointlio/odom for indoor testing of the RTK map-localization path.'
    )
    use_um981_arg = DeclareLaunchArgument(
        'use_um981',
        default_value='false',
        description='Start UM981 ROS node and publish GNSS GGA as /fix. UM981 IMU/INS output is not used.'
    )
    use_nav2_arg = DeclareLaunchArgument('use_nav2', default_value='true')
    use_coverage_planner_arg = DeclareLaunchArgument(
        'use_coverage_planner',
        default_value='false',
        description='Start the Fields2Cover ROS2 planner for a configured map-frame work area.'
    )
    launch_outdoor_rviz_arg = DeclareLaunchArgument('launch_outdoor_rviz', default_value='true')

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
            'unilidar_l2_ros2_no_pcd.yaml'),
        description='Point-LIO config file. Navigation defaults to no PCD saving.'
    )
    imu_quaternion_order_arg = DeclareLaunchArgument('imu_quaternion_order', default_value='wxyz')
    imu_angular_velocity_scale_arg = DeclareLaunchArgument(
        'imu_angular_velocity_scale', default_value='0.017453292519943295')
    imu_linear_acceleration_scale_arg = DeclareLaunchArgument('imu_linear_acceleration_scale', default_value='1.0')
    use_static_pointlio_pose_arg = DeclareLaunchArgument('use_static_pointlio_pose', default_value='false')
    use_map_to_camera_init_adapter_arg = DeclareLaunchArgument(
        'use_map_to_camera_init_adapter',
        default_value='true',
        description='Publish static map -> camera_init from the SLAM adapter. Set false when use_rtk_map_localizer is true.'
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
    map_to_camera_init_x_arg = DeclareLaunchArgument(
        'map_to_camera_init_x',
        default_value='0.0',
        description='Static map -> camera_init x offset used to align a prebuilt stage5 map with the current Point-LIO session.'
    )
    map_to_camera_init_y_arg = DeclareLaunchArgument(
        'map_to_camera_init_y',
        default_value='0.0',
        description='Static map -> camera_init y offset used to align a prebuilt stage5 map with the current Point-LIO session.'
    )
    map_to_camera_init_z_arg = DeclareLaunchArgument('map_to_camera_init_z', default_value='0.0')
    map_to_camera_init_roll_arg = DeclareLaunchArgument('map_to_camera_init_roll', default_value='0.0')
    map_to_camera_init_pitch_arg = DeclareLaunchArgument('map_to_camera_init_pitch', default_value='0.0')
    map_to_camera_init_yaw_arg = DeclareLaunchArgument(
        'map_to_camera_init_yaw',
        default_value='0.0',
        description='Static map -> camera_init yaw offset in radians used to align a prebuilt stage5 map with the current Point-LIO session.'
    )

    patchwork_cloud_topic_arg = DeclareLaunchArgument(
        'patchwork_cloud_topic',
        default_value='/unilidar/cloud',
        description='Input PointCloud2 topic for Patchwork++ ground segmentation.'
    )
    patchwork_sensor_height_arg = DeclareLaunchArgument(
        'patchwork_sensor_height',
        default_value='0.75',
        description='Approximate lidar mounting height in meters for Patchwork++.'
    )
    patchwork_min_r_arg = DeclareLaunchArgument('patchwork_min_r', default_value='0.2')
    patchwork_max_r_arg = DeclareLaunchArgument('patchwork_max_r', default_value='40.0')
    patchwork_log_every_n_arg = DeclareLaunchArgument('patchwork_log_every_n', default_value='60')
    min_ground_points_arg = DeclareLaunchArgument(
        'min_ground_points',
        default_value='100',
        description='Minimum Patchwork++ ground points required before elevation mapping uses the segmented ground cloud.'
    )

    segmented_map_dir_arg = DeclareLaunchArgument(
        'segmented_map_dir',
        default_value=default_map_session_dir,
        description='Timestamped directory containing ground_map.pcd and nonground_map.pcd; defaults to latest complete run.'
    )
    ground_map_filename_arg = DeclareLaunchArgument(
        'ground_map_filename', default_value='ground_map.pcd')
    nonground_map_filename_arg = DeclareLaunchArgument(
        'nonground_map_filename', default_value='nonground_map.pcd')
    map_metadata_path_arg = DeclareLaunchArgument(
        'map_metadata_path',
        default_value=os.path.join(default_map_session_dir, 'map_metadata.yaml'),
        description='Offline map georeference metadata from the selected timestamped map directory.'
    )
    fix_topic_arg = DeclareLaunchArgument('fix_topic', default_value='/fix')
    um981_port_arg = DeclareLaunchArgument(
        'um981_port',
        default_value='/dev/ttyUSB0',
        description='UM981 USB serial port. Prefer the /dev/serial/by-id/... path when it exists.'
    )
    um981_baud_arg = DeclareLaunchArgument('um981_baud', default_value='115200')
    um981_frame_id_arg = DeclareLaunchArgument('um981_frame_id', default_value='rtk_antenna')
    rtk_map_to_camera_init_yaw_arg = DeclareLaunchArgument(
        'rtk_map_to_camera_init_yaw',
        default_value='0.0',
        description='Initial yaw for map -> camera_init. Single-antenna RTK cannot estimate this automatically.'
    )
    use_um981_heading_arg = DeclareLaunchArgument(
        'use_um981_heading',
        default_value='false',
        description='Use UM981 INS heading topic to initialize map -> camera_init yaw.'
    )
    um981_heading_topic_arg = DeclareLaunchArgument(
        'um981_heading_topic',
        default_value='/um981/heading',
        description='std_msgs/Float64 heading in degrees, north-clockwise/east-positive.'
    )
    um981_heading_offset_deg_arg = DeclareLaunchArgument(
        'um981_heading_offset_deg',
        default_value='0.0',
        description='Offset from UM981 heading to robot/Point-LIO forward heading, in degrees.'
    )
    nav_map_resolution_arg = DeclareLaunchArgument('nav_map_resolution', default_value='0.10')
    nav2_start_delay_arg = DeclareLaunchArgument('nav2_start_delay', default_value='25.0')
    coverage_area_file_arg = DeclareLaunchArgument(
        'coverage_area_file',
        default_value=os.path.join(bringup_share, 'config', 'coverage_test_area.yaml'))
    coverage_output_file_arg = DeclareLaunchArgument(
        'coverage_output_file',
        default_value='~/.ros/golf_mower/coverage_path.yaml')
    coverage_dry_run_arg = DeclareLaunchArgument('coverage_dry_run', default_value='true')
    coverage_path_pose_spacing_arg = DeclareLaunchArgument(
        'coverage_path_pose_spacing', default_value='0.10')
    coverage_nav_waypoint_spacing_arg = DeclareLaunchArgument(
        'coverage_nav_waypoint_spacing', default_value='0.75')

    stage2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage2_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'use_elevation': LaunchConfiguration('use_elevation'),
            'use_cuda_elevation': LaunchConfiguration('use_cuda_elevation'),
            'use_grid_converter': LaunchConfiguration('use_grid_converter'),
            'launch_outdoor_rviz': LaunchConfiguration('launch_outdoor_rviz'),
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
            'use_static_pointlio_pose': LaunchConfiguration('use_static_pointlio_pose'),
            'use_map_to_camera_init_adapter': LaunchConfiguration('use_map_to_camera_init_adapter'),
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
            'map_to_camera_init_x': LaunchConfiguration('map_to_camera_init_x'),
            'map_to_camera_init_y': LaunchConfiguration('map_to_camera_init_y'),
            'map_to_camera_init_z': LaunchConfiguration('map_to_camera_init_z'),
            'map_to_camera_init_roll': LaunchConfiguration('map_to_camera_init_roll'),
            'map_to_camera_init_pitch': LaunchConfiguration('map_to_camera_init_pitch'),
            'map_to_camera_init_yaw': LaunchConfiguration('map_to_camera_init_yaw'),
            'elevation_unitree_config': elevation_config,
            'outdoor_rviz_config': rviz_config,
        }.items(),
    )

    segmented_map = Node(
        package='golf_mower_bringup',
        executable='pcd_to_occupancy_grid.py',
        name='segmented_pcd_to_occupancy_grid',
        output='screen',
        parameters=[{
            'free_pcd_path': PathJoinSubstitution([
                LaunchConfiguration('segmented_map_dir'), LaunchConfiguration('ground_map_filename')]),
            'occupied_pcd_path': PathJoinSubstitution([
                LaunchConfiguration('segmented_map_dir'), LaunchConfiguration('nonground_map_filename')]),
            'map_topic': '/map',
            'frame_id': 'map',
            'resolution': ParameterValue(LaunchConfiguration('nav_map_resolution'), value_type=float),
            'unknown_as_free': False,
            'publish_period_sec': 5.0,
        }],
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
            'sensor_height': ParameterValue(LaunchConfiguration('patchwork_sensor_height'), value_type=float),
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
            'frame_id': 'rtk_antenna',
            'metadata_path': LaunchConfiguration('map_metadata_path'),
            'position_covariance_m2': 0.04,
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

    rtk_map_localizer = Node(
        package='golf_mower_bringup',
        executable='rtk_map_localizer.py',
        name='rtk_map_localizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rtk_map_localizer')),
        parameters=[{
            'metadata_path': LaunchConfiguration('map_metadata_path'),
            'fix_topic': LaunchConfiguration('fix_topic'),
            'odom_topic': '/pointlio/odom',
            'map_frame': 'map',
            'camera_init_frame': 'camera_init',
            'map_to_camera_init_yaw': ParameterValue(
                LaunchConfiguration('rtk_map_to_camera_init_yaw'), value_type=float),
            'use_heading_topic': ParameterValue(
                LaunchConfiguration('use_um981_heading'), value_type=bool),
            'heading_topic': LaunchConfiguration('um981_heading_topic'),
            'heading_offset_deg': ParameterValue(
                LaunchConfiguration('um981_heading_offset_deg'), value_type=float),
            'publish_once': True,
            'min_fix_status': 0,
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
                    'params_file': nav2_params,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items(),
            )
        ],
    )

    coverage_planner = Node(
        package='golf_mower_bringup',
        executable='coverage_planner_node',
        name='coverage_planner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_coverage_planner')),
        parameters=[{
            'area_file': LaunchConfiguration('coverage_area_file'),
            'output_file': LaunchConfiguration('coverage_output_file'),
            'dry_run': ParameterValue(
                LaunchConfiguration('coverage_dry_run'), value_type=bool),
            'path_pose_spacing': ParameterValue(
                LaunchConfiguration('coverage_path_pose_spacing'), value_type=float),
            'nav_waypoint_spacing': ParameterValue(
                LaunchConfiguration('coverage_nav_waypoint_spacing'), value_type=float),
        }],
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_elevation_arg,
        use_cuda_elevation_arg,
        use_grid_converter_arg,
        use_patchwork_arg,
        use_ground_fallback_arg,
        allow_offline_ground_recovery_arg,
        use_rtk_map_localizer_arg,
        use_fake_rtk_arg,
        use_um981_arg,
        use_nav2_arg,
        use_coverage_planner_arg,
        launch_outdoor_rviz_arg,
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
        use_static_pointlio_pose_arg,
        use_map_to_camera_init_adapter_arg,
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
        map_to_camera_init_x_arg,
        map_to_camera_init_y_arg,
        map_to_camera_init_z_arg,
        map_to_camera_init_roll_arg,
        map_to_camera_init_pitch_arg,
        map_to_camera_init_yaw_arg,
        patchwork_cloud_topic_arg,
        patchwork_sensor_height_arg,
        patchwork_min_r_arg,
        patchwork_max_r_arg,
        patchwork_log_every_n_arg,
        min_ground_points_arg,
        segmented_map_dir_arg,
        ground_map_filename_arg,
        nonground_map_filename_arg,
        map_metadata_path_arg,
        fix_topic_arg,
        um981_port_arg,
        um981_baud_arg,
        um981_frame_id_arg,
        rtk_map_to_camera_init_yaw_arg,
        use_um981_heading_arg,
        um981_heading_topic_arg,
        um981_heading_offset_deg_arg,
        nav_map_resolution_arg,
        nav2_start_delay_arg,
        coverage_area_file_arg,
        coverage_output_file_arg,
        coverage_dry_run_arg,
        coverage_path_pose_spacing_arg,
        coverage_nav_waypoint_spacing_arg,
        OpaqueFunction(function=_validate_ground_map),
        stage2,
        segmented_map,
        patchwork,
        ground_fallback,
        fake_rtk,
        um981_node,
        rtk_map_localizer,
        nav2,
        coverage_planner,
    ])
