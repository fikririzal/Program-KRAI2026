import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_ust20lx')
    params_file = os.path.join(pkg_dir, 'config', 'ust20lx_params.yaml')

    ip_address = LaunchConfiguration('ip_address')
    ip_port    = LaunchConfiguration('ip_port')
    frame_id   = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('ip_address', default_value='192.168.0.100',
                              description='IP address Hokuyo UST-20LX'),
        DeclareLaunchArgument('ip_port', default_value='10940',
                              description='Port Ethernet UST-20LX'),
        DeclareLaunchArgument('frame_id', default_value='laser',
                              description='Frame ID untuk LaserScan'),

        # urg_node: driver resmi Hokuyo via urg_c
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'ip_address': ip_address,
                    'ip_port': ip_port,
                    'laser_frame_id': frame_id,
                },
            ],
        ),

        # scan_publisher: filter range & angle, republish ke /scan_filtered
        Node(
            package='lidar_ust20lx',
            executable='scan_publisher',
            name='scan_publisher',
            output='screen',
            parameters=[params_file],
        ),
    ])
