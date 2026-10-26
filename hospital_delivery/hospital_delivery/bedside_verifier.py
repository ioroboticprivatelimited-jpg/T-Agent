"""Confirms a meal reached the right patient before the order is closed.

Placeholder confirmation: waits for /verify/confirm (e.g. a bedside button, a
wristband QR scan or a nurse tap). Replace on_confirm with the real reader."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BedsideVerifier(Node):
    def __init__(self):
        super().__init__('bedside_verifier')
        self.declare_parameter('timeout_s', 60.0)
        self.pending = None
        self.timer = None
        self.pub = self.create_publisher(String, '/verify/result', 10)
        self.create_subscription(String, '/verify/request', self.on_request, 10)
        self.create_subscription(String, '/verify/confirm', self.on_confirm, 10)

    def on_request(self, m):
        self.pending = json.loads(m.data)       # {"order": id, "bed": "bed_2"}
        self.timer = self.create_timer(
            self.get_parameter('timeout_s').value, self.on_timeout)
        self.get_logger().info(f"waiting for confirmation at {self.pending['bed']}")

    def on_confirm(self, m):
        # {"bed": "bed_2", "patient": "..."}; match the bed so a wrong room fails
        if not self.pending:
            return
        ok = json.loads(m.data).get('bed') == self.pending['bed']
        self.finish(ok)

    def on_timeout(self):
        self.finish(False)

    def finish(self, ok):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.pub.publish(String(data=json.dumps(
            {'order': self.pending['order'], 'ok': ok})))
        self.pending = None


def main():
    rclpy.init()
    rclpy.spin(BedsideVerifier())
    rclpy.shutdown()
