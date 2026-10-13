import threading
import struct
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

HEADER = bytes([0x55, 0x53])
PACKET_LEN = 11


class ImuNode(Node):
    def __init__(self):
        super().__init__('witmotion_imu_node')

        self.declare_parameter('port',     '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self._pub = self.create_publisher(Float32, 'imu_yaw', 10)
        self._buf = bytearray()

        self._ser = serial.Serial(port, baudrate, timeout=1.0)
        self._ser.reset_input_buffer()

        threading.Thread(target=self._read_loop, daemon=True).start()
        self.get_logger().info(f'Witmotion IMU ready {port}@{baudrate} → /imu_yaw')

    def _read_loop(self):
        while rclpy.ok():
            try:
                chunk = self._ser.read(44)
                if not chunk:
                    continue
                self._buf.extend(chunk)

                # cari header 0x55 0x53 dalam buffer
                while len(self._buf) >= PACKET_LEN:
                    idx = self._buf.find(HEADER)
                    if idx == -1:
                        self._buf = self._buf[-1:]
                        break
                    if idx > 0:
                        del self._buf[:idx]

                    if len(self._buf) < PACKET_LEN:
                        break

                    packet = self._buf[:PACKET_LEN]
                    # yaw: byte 6 (LSB) dan 7 (MSB)
                    # balik byte order agar kanan = + (65535 - raw)
                    yaw_raw = 65535 - struct.unpack('<H', packet[6:8])[0]
                    yaw = yaw_raw / 32768.0 * 180.0

                    # normalize 0-360 → -180..+180
                    if yaw > 180.0:
                        yaw -= 360.0


                    msg = Float32()
                    msg.data = yaw
                    self._pub.publish(msg)

                    del self._buf[:PACKET_LEN]

            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break
            except Exception:
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
