"""Estimates crowd density ahead of the robot from the LiDAR scan.

C = share of valid beams in a forward sector that hit something closer than
`range_m`. It is a cheap proxy (people, trolleys and walls all count), so
tune `range_m` and the sector to your robot. Publishes JSON on /crowd/level:
{"c": 0..1, "stamp": <scan time, s>} so the agent can measure its own delay."""
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class CrowdMonitor(Node):
    def __init__(self):
        super().__init__('crowd_monitor')
        self.declare_parameter('range_m', 3.0)
        self.declare_parameter('sector_deg', 120.0)
        self.declare_parameter('smooth', 0.4)       # 0 = raw, 1 = frozen
        self.c = 0.0
        self.pub = self.create_publisher(String, '/crowd/level', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, m):
        half = math.radians(self.get_parameter('sector_deg').value) / 2.0
        lim = self.get_parameter('range_m').value
        hits = total = 0
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if abs(a) > half or not math.isfinite(r) or r < m.range_min:
                continue
            total += 1
            if r < lim:
                hits += 1
        if total == 0:
            return
        k = self.get_parameter('smooth').value
        self.c = k * self.c + (1.0 - k) * (hits / total)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pub.publish(String(data=json.dumps({'c': self.c, 'stamp': stamp})))


def main():
    rclpy.init()
    rclpy.spin(CrowdMonitor())
    rclpy.shutdown()
