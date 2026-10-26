"""Latches /safety/stop when the LiDAR sees an obstacle too close or goes silent,
or when the hardware e-stop topic is pressed. Independent of the planner."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('min_range_m', 0.35)
        self.declare_parameter('scan_timeout_s', 0.5)
        self.last_scan = self.get_clock().now()
        self.estop = False
        self.stop = False
        self.pub = self.create_publisher(Bool, '/safety/stop', 10)
        self.zero = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Bool, '/estop', self.on_estop, 10)
        self.create_subscription(Bool, '/safety/reset', self.on_reset, 10)
        self.create_timer(0.1, self.tick)

    def on_scan(self, m):
        self.last_scan = self.get_clock().now()
        lim = self.get_parameter('min_range_m').value
        near = [r for r in m.ranges if math.isfinite(r) and m.range_min < r < lim]
        if near:
            self.stop = True

    def on_estop(self, m):
        self.estop = m.data
        if m.data:
            self.stop = True

    def on_reset(self, m):
        if m.data and not self.estop:
            self.stop = False        # latched until a person resets it

    def tick(self):
        age = (self.get_clock().now() - self.last_scan).nanoseconds * 1e-9
        if age > self.get_parameter('scan_timeout_s').value:
            self.stop = True
        self.pub.publish(Bool(data=self.stop))
        if self.stop:
            self.zero.publish(Twist())   # zero velocity, muxed above Nav2


def main():
    rclpy.init()
    rclpy.spin(SafetySupervisor())
    rclpy.shutdown()
