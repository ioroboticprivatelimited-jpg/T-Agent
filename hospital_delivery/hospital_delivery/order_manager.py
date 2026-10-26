"""Receives meal orders and publishes them as JSON on /orders/new."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.pub = self.create_publisher(String, '/orders/new', 10)
        # staff dashboard / HIS bridge publishes raw orders here
        self.create_subscription(String, '/orders/raw', self.on_raw, 10)
        self.next_id = 1

    def on_raw(self, msg):
        try:
            raw = json.loads(msg.data)       # {"bed": "bed_2", "urgency": 0-1, "diet": "..."}
            order = {
                'id': self.next_id,
                'bed': str(raw['bed']),
                'urgency': float(raw.get('urgency', 0.5)),
                'diet': raw.get('diet', 'standard'),
                'created': time.time(),
                'deadline': float(raw.get('deadline_s', 1800.0)),
            }
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f'rejected order: {e}')
            return
        self.next_id += 1
        self.pub.publish(String(data=json.dumps(order)))
        self.get_logger().info(f"order {order['id']} -> {order['bed']}")


def main():
    rclpy.init()
    rclpy.spin(OrderManager())
    rclpy.shutdown()
