#!/usr/bin/env python3
"""
Visual servo node — center objek di frame, jaga jarak 30 cm.

Subscribes : /camera/detections  (std_msgs/String, JSON)
Publishes  : /cmd_vel            (geometry_msgs/Twist)
             /servo/target_xy    (std_msgs/String)  log x,y,depth
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Twist


class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        # ── params ──────────────────────────────────────────────────────────
        self.declare_parameter('frame_w',      1280)
        self.declare_parameter('frame_h',      720)
        self.declare_parameter('target_depth', 0.30)   # meter
        self.declare_parameter('kp_yaw',       0.0012) # pixel error → rad/s
        self.declare_parameter('kp_fwd',       0.5)    # meter error → m/s
        self.declare_parameter('max_wz',       0.1)    # m/s (vy strafe)
        self.declare_parameter('max_vx',       0.1)    # m/s (vx forward)
        self.declare_parameter('tol_x',        25)     # pixel tolerance
        self.declare_parameter('tol_depth',    0.02)   # meter tolerance
        self.declare_parameter('target_label', '')     # kosong = objek pertama

        self._fw        = self.get_parameter('frame_w').value
        self._fh        = self.get_parameter('frame_h').value
        self._t_depth   = self.get_parameter('target_depth').value
        self._kp_yaw    = self.get_parameter('kp_yaw').value
        self._kp_fwd    = self.get_parameter('kp_fwd').value
        self._max_wz    = self.get_parameter('max_wz').value
        self._max_vx    = self.get_parameter('max_vx').value
        self._tol_x     = self.get_parameter('tol_x').value
        self._tol_depth = self.get_parameter('tol_depth').value
        self._label     = self.get_parameter('target_label').value

        self._cx = self._fw // 2   # pusat frame x
        self._cy = self._fh // 2   # pusat frame y

        # ── pub/sub ─────────────────────────────────────────────────────────
        self._cmd_pub     = self.create_publisher(Twist,  '/cmd_vel',         10)
        self._info_pub    = self.create_publisher(String, '/servo/target_xy',  10)
        self._lifter1_pub = self.create_publisher(Int16,  '/lifter_robot1',   10)
        self._lifter2_pub = self.create_publisher(Int16,  '/lifter_robot2',   10)

        self._lifter_sent = False

        self.create_subscription(String, '/camera/detections', self._cb, 10)

        self.get_logger().info(
            f'visual_servo_node ready  target_depth={self._t_depth}m  '
            f'frame={self._fw}x{self._fh}'
        )

    # ── callback ─────────────────────────────────────────────────────────────

    def _cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not detections:
            self._stop()
            return

        # pilih target: label tertentu atau objek terdekat
        if self._label:
            candidates = [d for d in detections if d.get('label') == self._label]
        else:
            candidates = detections

        if not candidates:
            self._stop()
            return

        # ambil yang terdekat (depth terkecil)
        target = min(candidates, key=lambda d: d.get('depth_m', 999))

        # kirim lifter sekali sebelum servo aktif pertama kali
        if not self._lifter_sent:
            val = Int16(data=-500)
            self._lifter1_pub.publish(val)
            self._lifter2_pub.publish(val)
            self._lifter_sent = True
            self.get_logger().info('lifter1 & lifter2 → -500')

        cx, cy  = target['center']
        depth   = target.get('depth_m', 0.0)
        label   = target.get('label', '?')

        # ── error ────────────────────────────────────────────────────────────
        err_x     = self._cx - cx          # positif = objek di kiri → geser kiri (vy+)
        err_depth = depth - self._t_depth  # positif = terlalu jauh  → maju (vx+)

        # ── kontrol P ────────────────────────────────────────────────────────
        vy = self._clamp(self._kp_yaw * err_x,     self._max_wz)   # x → vy (strafe)
        vx = self._clamp(self._kp_fwd * err_depth, self._max_vx)   # depth → vx (maju)

        done_x     = abs(err_x)     < self._tol_x
        done_depth = abs(err_depth) < self._tol_depth

        cmd = Twist()
        cmd.linear.x  = 0.0 if done_depth else vx
        cmd.linear.y  = 0.0 if done_x     else vy
        cmd.angular.z = 0.0   # tidak kirim yaw
        self._cmd_pub.publish(cmd)

        # ── log x,y,depth ────────────────────────────────────────────────────
        info = (
            f'{label}  '
            f'x={cx}(err={err_x:+d}px→vy={vy:.2f})  '
            f'y={cy}  '
            f'depth={depth:.3f}m(err={err_depth:+.3f}m→vx={vx:.2f})  '
            f'{"CENTER✓ " if done_x else ""}'
            f'{"DEPTH✓" if done_depth else ""}'
        )
        self.get_logger().info(info, throttle_duration_sec=0.2)
        self._info_pub.publish(String(data=info))

    def _stop(self):
        self._cmd_pub.publish(Twist())

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, v))


# ── entry ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
