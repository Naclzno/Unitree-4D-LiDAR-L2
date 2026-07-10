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
    indoor_slam_launch = os.path.join(
        bringup_share, 'launch', 'indoor_slam_test.launch.py')

    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_pointlio_arg = DeclareLaunchArgument('use_pointlio', default_value='true')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    use_fake_rtk_arg = DeclareLaunchArgument('use_fake_rtk', default_value='true')
    use_um981_arg = DeclareLaunchArgument(
        'use_um981',
        default_value='false',
        description='Start the UM981 ROS node for GNSS /fix topic validation. GNSS fix may be unavailable indoors.'
    )

    initialize_type_arg = DeclareLaunchArgument('initialize_type', default_value='2')
    work_mode_arg = DeclareLaunchArgument('work_mode', default_value='0')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='4000000')
    start_lidar_rotation_arg = DeclareLaunchArgument('start_lidar_rotation', default_value='true')
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument('reset_lidar_after_set_mode', default_value='true')

    datum_lat_arg = DeclareLaunchArgument('datum_lat_deg', default_value='39.771981522833336')
    datum_lon_arg = DeclareLaunchArgument('datum_lon_deg', default_value='116.353032825')
    datum_alt_arg = DeclareLaunchArgument('datum_alt_m', default_value='79.0')
    fake_fix_topic_arg = DeclareLaunchArgument('fake_fix_topic', default_value='/fix')

    um981_port_arg = DeclareLaunchArgument(
        'um981_port',
        default_value='/dev/ttyUSB0')
    um981_baud_arg = DeclareLaunchArgument('um981_baud', default_value='115200')

    indoor_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(indoor_slam_launch),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_pointlio': LaunchConfiguration('use_pointlio'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_tf_adapter': 'true',
            'use_lidar_tf_adapter': 'true',
            'initialize_type': LaunchConfiguration('initialize_type'),
            'work_mode': LaunchConfiguration('work_mode'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'start_lidar_rotation': LaunchConfiguration('start_lidar_rotation'),
            'reset_lidar_after_set_mode': LaunchConfiguration('reset_lidar_after_set_mode'),
        }.items(),
    )

    fake_rtk = Node(
        package='golf_mower_bringup',
        executable='fake_rtk_from_odom.py',
        name='fake_rtk_from_odom',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_rtk')),
        parameters=[{
            'odom_topic': '/pointlio/odom',
            'fix_topic': LaunchConfiguration('fake_fix_topic'),
            'frame_id': 'rtk_antenna',
            'datum_lat_deg': ParameterValue(LaunchConfiguration('datum_lat_deg'), value_type=float),
            'datum_lon_deg': ParameterValue(LaunchConfiguration('datum_lon_deg'), value_type=float),
            'datum_alt_m': ParameterValue(LaunchConfiguration('datum_alt_m'), value_type=float),
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
            ('/fix', LaunchConfiguration('fake_fix_topic')),
        ],
        parameters=[{
            'port': LaunchConfiguration('um981_port'),
            'baud': ParameterValue(LaunchConfiguration('um981_baud'), value_type=int),
            'frame_id': 'rtk_antenna',
            'imu_frame_id': 'um981_imu',
            'commands': ['GNGGA 1'],
        }],
    )

    return LaunchDescription([
        use_lidar_arg,
        use_pointlio_arg,
        launch_rviz_arg,
        use_fake_rtk_arg,
        use_um981_arg,
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        datum_lat_arg,
        datum_lon_arg,
        datum_alt_arg,
        fake_fix_topic_arg,
        um981_port_arg,
        um981_baud_arg,
        indoor_slam,
        fake_rtk,
        um981_node,
    ])
