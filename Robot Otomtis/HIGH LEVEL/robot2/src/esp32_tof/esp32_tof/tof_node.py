import threading
import serial

import rclpy
from rclpy.node import Node
from sensor_tof.msg import TofGripper


class Esp32TofNode(Node):
    def __init__(self):
        super().__init__('esp32_tof_node')

        self.declare_parameter('port',     '/dev/ttyUSB2')
        self.declare_parameter('baudrate', 115200)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self._pub = self.create_publisher(TofGripper, 'tof/gripper', 10)

        self._ser = serial.Serial(port, baudrate, timeout=1.0)
        self._ser.reset_input_buffer()

        threading.Thread(target=self._read_loop, daemon=True).start()
        self.get_logger().info(f'ESP32 TOF gripper ready {port}@{baudrate} → /tof/gripper')

    def _read_loop(self):
        while rclpy.ok():
            try:
                line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')
                if len(parts) != 8:
                    continue

                dist = [int(p) for p in parts]

                msg = TofGripper()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'tof_gripper'
                msg.dist            = dist

                self._pub.publish(msg)

            except (ValueError, serial.SerialException):
                continue

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32TofNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
