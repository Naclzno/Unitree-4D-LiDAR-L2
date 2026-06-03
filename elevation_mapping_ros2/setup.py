from setuptools import find_packages, setup

package_name = 'elevation_mapping_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@localhost',
    description='CPU-only ROS2 elevation mapping node compatible with elevation_mapping_cupy topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevation_mapping_node.py = elevation_mapping_ros2.elevation_mapping_node:main',
        ],
    },
)
