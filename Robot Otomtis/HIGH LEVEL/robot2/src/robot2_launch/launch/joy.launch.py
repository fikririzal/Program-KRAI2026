import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ust20lx_launch = os.path.join(
        get_package_share_directory('lidar_ust20lx'), 'launch', 'ust20lx.launch.py'
    )

    return LaunchDescription([

        # ── lidar UST-20LX (hadap belakang, 180°) ────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ust20lx_launch),
        ),

        # ── RANSAC wall detector ──────────────────────────────────────────────
        # Publishes: /walls [belakang, kiri, kanan] dalam meter
        Node(
            package='wall_detector',
            executable='wall_node',
            name='wall_node',
            output='screen',
            parameters=[{
                'scan_topic':   '/scan_filtered',
                'walls_topic':  '/walls',
                'ransac_iter':  80,
                'dist_thr':     0.05,
                'min_inliers':  8,
            }],
        ),

        # # ── BNO055 IMU via Arduino Nano (nonaktif) ────────────────────────────
        # Node(
        #     package='bno055_imu',
        #     executable='imu_node',
        #     name='bno055_imu_node',
        #     output='screen',
        #     parameters=[{
        #         'port':     '/dev/ttyUSB1',
        #         'baudrate': 115200,
        #     }],
        # ),

        # ── Witmotion HCT IMU → /imu_yaw (Float32) ───────────────────────────
        Node(
            package='witmotion_imu',
            executable='imu_node',
            name='witmotion_imu_node',
            output='screen',
            parameters=[{
                'port':     '/dev/ttyACM1',
                'baudrate': 115200,
            }],
        ),

        # ── joystick driver ───────────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),

        # ── hardware: STM32 via ttyUSB0 ──────────────────────────────────────
        Node(
            package='hardware1_com',
            executable='hw_node',
            name='hw_node',
            output='screen',
            parameters=[{
                'port':     '/dev/ttyUSB0',
                'baudrate': 115200,
            }],
        ),

        # ── sensor TOF ────────────────────────────────────────────────────────
        # Publishes : /tof/array (sensor_tof/TofArray)
        Node(
            package='sensor_tof',
            executable='tof_node',
            name='tof_node',
            output='screen',
            parameters=[{
                'ports':    ['/dev/ttyACM0'],
                'baudrate': 921600,
            }],
        ),

        # ── ESP32 TOF gripper ─────────────────────────────────────────────────
        # Publishes : /tof/gripper (sensor_tof/TofGripper), 8 sensor via USB CSV
        Node(
            package='esp32_tof',
            executable='tof_node',
            name='esp32_tof_node',
            output='screen',
            parameters=[{
                'port':     '/dev/ttyUSB1',
                'baudrate': 115200,
            }],
        ),

        # # ── RealSense D435i + YOLOv8n TensorRT ───────────────────────────────
        # # Publishes: /camera/color/image, /camera/depth/image, /camera/detections
        # Node(
        #     package='realsense',
        #     executable='realsense_node.py',
        #     name='realsense_node',
        #     output='screen',
        #     arguments=['model:=kfsnew.pt', 'conf:=0.8', 'imgsz:=640'],
        # ),

        # # ── monitor terminal ──────────────────────────────────────────────────
        # Node(
        #     package='robot2_monitor',
        #     executable='monitor_node',
        #     name='monitor_node',
        #     output='screen',
        # ),

        # ── eira interface (GUI) ──────────────────────────────────────────────
        Node(
            package='eira_interface',
            executable='interface_node',
            name='interface_node',
            output='screen',
        ),

        Node(
            package='eira_interface',
            executable='reciverweb',
            name='reciverweb',
            output='screen',
        ),

        Node(
            package='eira_intelligent',
            executable='intelligent_node',
            name='intelligent_node',
            output='screen',
        ),

        # # ── sekuensial robot (tulis logikamu di seq_node.cpp) ─────────────────
        # Node(
        #     package='robot2_master',
        #     executable='seq_node',
        #     name='seq_node',
        #     output='screen',
        # ),

    ])
