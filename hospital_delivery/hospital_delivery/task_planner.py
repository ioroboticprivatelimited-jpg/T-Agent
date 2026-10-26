"""Scores queued orders, S_i = w_u*U_i + w_t*T_i - w_d*D_i, and dispatches the best.

U_i urgency (0-1), T_i time pressure (0-1, grows as the deadline nears),
D_i distance to the bed, normalised (0-1). A battery constraint sends the
robot to the dock when charge falls below battery_floor.
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        self.declare_parameter('w_urgency', 0.5)
        self.declare_parameter('w_time', 0.3)
        self.declare_parameter('w_distance', 0.2)
        self.declare_parameter('battery_floor', 0.20)
        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])
        self.queue = []
        self.busy = False
        self.battery = 1.0
        self.pose = (0.0, 0.0)
        self.beds = {}                        # bed_id -> (x, y), from /nav/beds
        self.create_subscription(String, '/orders/new', self.on_order, 10)
        self.create_subscription(String, '/nav/beds', self.on_beds, 10)
        self.create_subscription(String, '/nav/status', self.on_status, 10)
        self.create_subscription(Float32, '/battery', self.on_batt, 10)
        self.create_subscription(String, '/robot/pose2d', self.on_pose, 10)
        self.goal_pub = self.create_publisher(String, '/nav/goal', 10)
        self.create_timer(1.0, self.tick)

    def on_order(self, m):
        self.queue.append(json.loads(m.data))

    def on_beds(self, m):
        self.beds = {k: tuple(v) for k, v in json.loads(m.data).items()}

    def on_batt(self, m):
        self.battery = m.data

    def on_pose(self, m):
        x, y = json.loads(m.data)
        self.pose = (x, y)

    def on_status(self, m):
        # {"state": "done" | "failed" | "idle" | "active", "order": id}
        if json.loads(m.data)['state'] in ('done', 'failed', 'idle'):
            self.busy = False

    def score(self, o):
        wu = self.get_parameter('w_urgency').value
        wt = self.get_parameter('w_time').value
        wd = self.get_parameter('w_distance').value
        u = o['urgency']
        waited = time.time() - o['created']
        t = min(1.0, waited / max(o['deadline'], 1.0))
        bx, by = self.beds.get(o['bed'], self.pose)
        d = min(1.0, math.hypot(bx - self.pose[0], by - self.pose[1]) / 30.0)
        return wu * u + wt * t - wd * d

    def tick(self):
        if self.busy:
            return
        if self.battery < self.get_parameter('battery_floor').value:
            self.dispatch({'kind': 'dock', 'order': 0})
            return
        if not self.queue:
            return
        best = max(self.queue, key=self.score)
        self.queue.remove(best)
        self.dispatch({'kind': 'deliver', 'order': best['id'], 'bed': best['bed']})

    def dispatch(self, goal):
        self.busy = True
        self.goal_pub.publish(String(data=json.dumps(goal)))
        self.get_logger().info(f'dispatch {goal}')


def main():
    rclpy.init()
    rclpy.spin(TaskPlanner())
    rclpy.shutdown()
