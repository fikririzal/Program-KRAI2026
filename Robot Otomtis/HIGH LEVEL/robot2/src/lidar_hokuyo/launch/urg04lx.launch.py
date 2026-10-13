from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Hokuyo URG-04LX-UG01'
        ),
        DeclareLaunchArgument(
            'serial_baud',
            default_value='115200',
            description='Serial baud rate'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame ID for LaserScan'
        ),
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baud': serial_baud,
                'frame_id': frame_id,
            }]
        ),
    ])
