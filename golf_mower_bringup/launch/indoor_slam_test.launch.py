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
    point_lio_share = get_package_share_directory('point_lio_unilidar')

    point_lio_launch = os.path.join(
        point_lio_share, 'launch', 'mapping_unilidar_l2.py')
    rviz_config = os.path.join(
        point_lio_share, 'rviz_cfg', 'loam_unilidar_display.rviz')

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
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz with the Point-LIO display config.'
    )
    use_tf_adapter_arg = DeclareLaunchArgument(
        'use_tf_adapter',
        default_value='true',
        description='Publish temporary map/base frames for indoor bench testing.'
    )
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
            {'use_system_timestamp': True},
            {'start_lidar_rotation': ParameterValue(
                LaunchConfiguration('start_lidar_rotation'), value_type=bool)},
            {'reset_lidar_after_set_mode': ParameterValue(
                LaunchConfiguration('reset_lidar_after_set_mode'), value_type=bool)},
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
        }.items(),
    )

    map_to_pointlio_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_tf_adapter')),
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
        condition=IfCondition(LaunchConfiguration('use_tf_adapter')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'aft_mapped',
            '--child-frame-id', 'base_link',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_indoor_slam',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        use_rviz_arg,
        use_tf_adapter_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        save_cloud_txt_arg,
        cloud_txt_save_mode_arg,
        cloud_txt_path_arg,
        cloud_txt_dir_arg,
        cloud_txt_save_every_n_arg,
        lidar_node,
        pointlio,
        map_to_pointlio_map_tf,
        pointlio_body_to_base_tf,
        rviz_node,
    ])
