import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ImuNode(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        self.declare_parameter('port',     '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self._pub = self.create_publisher(Float32, 'imu_yaw', 10)

        self._ser = serial.Serial(port, baudrate, timeout=1.0)
        self._ser.reset_input_buffer()

        threading.Thread(target=self._read_loop, daemon=True).start()
        self.get_logger().info(f'IMU ready {port}@{baudrate} → /imu_yaw')

    def _read_loop(self):
        while rclpy.ok():
            try:
                line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                msg = Float32()
                msg.data = float(line)
                self._pub.publish(msg)
            except (ValueError, serial.SerialException):
                continue

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
