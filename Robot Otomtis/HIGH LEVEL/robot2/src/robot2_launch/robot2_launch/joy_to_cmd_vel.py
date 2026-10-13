import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')

        # axis index
        self.declare_parameter('axis_linear_x', 1)   # left stick up/down
        self.declare_parameter('axis_linear_y', 0)   # left stick left/right
        self.declare_parameter('axis_angular_z', 2)  # right stick left/right

        # scale (m/s dan rad/s)
        self.declare_parameter('scale_linear_x', 1.0)
        self.declare_parameter('scale_linear_y', 1.0)
        self.declare_parameter('scale_angular_z', 1.0)

        self.ax_lx = self.get_parameter('axis_linear_x').value
        self.ax_ly = self.get_parameter('axis_linear_y').value
        self.ax_wz = self.get_parameter('axis_angular_z').value
        self.sc_lx = self.get_parameter('scale_linear_x').value
        self.sc_ly = self.get_parameter('scale_linear_y').value
        self.sc_wz = self.get_parameter('scale_angular_z').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)

    def joy_cb(self, msg: Joy):
        twist = Twist()
        if self.ax_lx < len(msg.axes):
            twist.linear.x = msg.axes[self.ax_lx] * self.sc_lx
        if self.ax_ly < len(msg.axes):
            twist.linear.y = msg.axes[self.ax_ly] * self.sc_ly
        if self.ax_wz < len(msg.axes):
            twist.angular.z = msg.axes[self.ax_wz] * self.sc_wz * -1.0
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()
