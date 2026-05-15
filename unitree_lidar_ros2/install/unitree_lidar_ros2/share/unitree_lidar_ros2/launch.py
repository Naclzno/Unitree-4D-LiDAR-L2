import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    save_cloud_txt_arg = DeclareLaunchArgument(
        'save_cloud_txt',
        default_value='false',
        description='Save parsed point cloud data to a txt file.'
    )
    cloud_txt_path_arg = DeclareLaunchArgument(
        'cloud_txt_path',
        default_value='/tmp/unitree_lidar_cloud.txt',
<<<<<<< HEAD
        description='Output txt file path for parsed point cloud data.'
    )
    cloud_txt_append_arg = DeclareLaunchArgument(
        'cloud_txt_append',
        default_value='true',
        description='Append to the txt file instead of overwriting it.'
=======
        description='Output txt file path used when cloud_txt_save_mode is overwrite_one_file.'
    )
    cloud_txt_save_mode_arg = DeclareLaunchArgument(
        'cloud_txt_save_mode',
        default_value='overwrite_one_file',
        description='Point cloud txt save mode: overwrite_one_file or separate_files.'
    )
    cloud_txt_dir_arg = DeclareLaunchArgument(
        'cloud_txt_dir',
        default_value='/tmp/unitree_lidar_cloud_frames',
        description='Output directory used when cloud_txt_save_mode is separate_files.'
>>>>>>> test1
    )
    cloud_txt_save_every_n_arg = DeclareLaunchArgument(
        'cloud_txt_save_every_n',
        default_value='1',
        description='Save one point cloud frame every N parsed frames.'
    )

    # Run unitree lidar
    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                
                {'initialize_type': 2},
                {'work_mode': 0},
                {'use_system_timestamp': True},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': '192.168.1.62'},
                {'local_port': 6201},
                {'local_ip': '192.168.1.2'},
                
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},

                {'save_cloud_txt': ParameterValue(LaunchConfiguration('save_cloud_txt'), value_type=bool)},
<<<<<<< HEAD
                {'cloud_txt_path': LaunchConfiguration('cloud_txt_path')},
                {'cloud_txt_append': ParameterValue(LaunchConfiguration('cloud_txt_append'), value_type=bool)},
=======
                {'cloud_txt_save_mode': LaunchConfiguration('cloud_txt_save_mode')},
                {'cloud_txt_path': LaunchConfiguration('cloud_txt_path')},
                {'cloud_txt_dir': LaunchConfiguration('cloud_txt_dir')},
>>>>>>> test1
                {'cloud_txt_save_every_n': ParameterValue(LaunchConfiguration('cloud_txt_save_every_n'), value_type=int)},
                ]
    )

    # Run Rviz
    package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
    print("rviz_config_file = " + rviz_config_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log'
    )
    return LaunchDescription([
        save_cloud_txt_arg,
<<<<<<< HEAD
        cloud_txt_path_arg,
        cloud_txt_append_arg,
=======
        cloud_txt_save_mode_arg,
        cloud_txt_path_arg,
        cloud_txt_dir_arg,
>>>>>>> test1
        cloud_txt_save_every_n_arg,
        node1,
        rviz_node,
    ])
