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
    stage1_launch = os.path.join(
        bringup_share, 'launch', 'outdoor_elevation_stage1.launch.py')

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    use_elevation_arg = DeclareLaunchArgument('use_elevation', default_value='true')
    use_grid_converter_arg = DeclareLaunchArgument('use_grid_converter', default_value='true')
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
    pointlio_config_file_arg = DeclareLaunchArgument(
        'pointlio_config_file',
        default_value=os.path.join(
            get_package_share_directory('point_lio_unilidar'),
            'config',
            'unilidar_l2_ros2_gravity_positive.yaml'),
        description='Point-LIO config file.'
    )
    imu_quaternion_order_arg = DeclareLaunchArgument(
        'imu_quaternion_order',
        default_value='wxyz',
        description='Order of Unitree SDK quaternion values: wxyz or xyzw.'
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
        default_value=os.path.join(bringup_share, 'config', 'elevation_unitree_l2_stage1.yaml'),
        description='Unitree L2 specific elevation_mapping_cupy parameter file.'
    )
    outdoor_rviz_config_arg = DeclareLaunchArgument(
        'outdoor_rviz_config',
        default_value=os.path.join(bringup_share, 'rviz', 'outdoor_elevation.rviz'),
        description='RViz config used by the outdoor elevation launch.'
    )

    grid_map_topic_arg = DeclareLaunchArgument(
        'grid_map_topic',
        default_value='/elevation_mapping_node/elevation_map_filter',
        description='Input GridMap topic from elevation_mapping_cupy.'
    )
    occupancy_grid_topic_arg = DeclareLaunchArgument(
        'occupancy_grid_topic',
        default_value='/elevation/traversability_grid',
        description='Output OccupancyGrid topic for traversability.'
    )
    grid_map_layer_arg = DeclareLaunchArgument(
        'grid_map_layer',
        default_value='traversability',
        description='GridMap layer used for occupancy conversion.'
    )
    free_threshold_arg = DeclareLaunchArgument(
        'free_threshold',
        default_value='0.70',
        description='Traversability value at or above this is free.'
    )
    occupied_threshold_arg = DeclareLaunchArgument(
        'occupied_threshold',
        default_value='0.45',
        description='Traversability value at or below this is occupied.'
    )

    stage1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage1_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'use_elevation': LaunchConfiguration('use_elevation'),
            'launch_outdoor_rviz': LaunchConfiguration('launch_outdoor_rviz'),
            'use_lidar_tf_adapter': LaunchConfiguration('use_lidar_tf_adapter'),
            'use_static_pointlio_pose': LaunchConfiguration('use_static_pointlio_pose'),
            'initialize_type': LaunchConfiguration('initialize_type'),
            'work_mode': LaunchConfiguration('work_mode'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'start_lidar_rotation': LaunchConfiguration('start_lidar_rotation'),
            'reset_lidar_after_set_mode': LaunchConfiguration('reset_lidar_after_set_mode'),
            'pointlio_config_file': LaunchConfiguration('pointlio_config_file'),
            'imu_quaternion_order': LaunchConfiguration('imu_quaternion_order'),
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
            'elevation_unitree_config': LaunchConfiguration('elevation_unitree_config'),
            'outdoor_rviz_config': LaunchConfiguration('outdoor_rviz_config'),
        }.items(),
    )

    grid_map_converter = Node(
        package='golf_mower_bringup',
        executable='grid_map_to_occupancy_grid.py',
        name='grid_map_to_occupancy_grid',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_grid_converter')),
        parameters=[{
            'grid_map_topic': LaunchConfiguration('grid_map_topic'),
            'occupancy_grid_topic': LaunchConfiguration('occupancy_grid_topic'),
            'layer': LaunchConfiguration('grid_map_layer'),
            'free_threshold': ParameterValue(LaunchConfiguration('free_threshold'), value_type=float),
            'occupied_threshold': ParameterValue(LaunchConfiguration('occupied_threshold'), value_type=float),
        }],
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_elevation_arg,
        use_grid_converter_arg,
        launch_outdoor_rviz_arg,
        use_lidar_tf_adapter_arg,
        use_static_pointlio_pose_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        pointlio_config_file_arg,
        imu_quaternion_order_arg,
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
        grid_map_topic_arg,
        occupancy_grid_topic_arg,
        grid_map_layer_arg,
        free_threshold_arg,
        occupied_threshold_arg,
        stage1,
        grid_map_converter,
    ])
