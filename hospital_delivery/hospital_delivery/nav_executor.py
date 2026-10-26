"""Turns /nav/goal into Nav2 NavigateToPose goals and reports status."""
import json
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool


class NavExecutor(Node):
    def __init__(self):
        super().__init__('nav_executor')
        self.declare_parameter('beds', ['bed_1'])
        self.beds = {}
        for b in self.get_parameter('beds').value:
            self.declare_parameter(b, [0.0, 0.0, 0.0])
            self.beds[b] = list(self.get_parameter(b).value)
        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status = self.create_publisher(String, '/nav/status', 10)
        self.bed_pub = self.create_publisher(String, '/nav/beds', 1)
        self.verify_req = self.create_publisher(String, '/verify/request', 10)
        self.create_subscription(String, '/nav/goal', self.on_goal, 10)
        self.create_subscription(Bool, '/safety/stop', self.on_stop, 10)
        self.create_subscription(String, '/verify/result', self.on_verify, 10)
        self.handle = None
        self.current = None
        self.xyz = None
        self.held = False
        self.create_subscription(Bool, '/nav/hold', self.on_hold, 10)
        self.create_timer(5.0, self.publish_beds)

    def publish_beds(self):
        self.bed_pub.publish(String(data=json.dumps(
            {k: v[:2] for k, v in self.beds.items()})))

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z = math.sin(yaw / 2)
        p.pose.orientation.w = math.cos(yaw / 2)
        return p

    def on_goal(self, m):
        g = json.loads(m.data)
        if g['kind'] == 'dock':
            xyz = list(self.get_parameter('dock_pose').value)
        elif g['bed'] in self.beds:
            xyz = self.beds[g['bed']]
        else:
            self.report('failed', g['order'])
            return
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.report('failed', g['order'])
            return
        self.current = g
        self.xyz = xyz
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(*xyz)
        self.report('active', g['order'])
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        self.handle = fut.result()
        if not self.handle.accepted:
            self.report('failed', self.current['order'])
            return
        self.handle.get_result_async().add_done_callback(self.on_result)

    def on_hold(self, m):
        if m.data and self.handle is not None and not self.held:
            self.held = True
            self.handle.cancel_goal_async()          # stop and wait
        elif not m.data and self.held:
            self.held = False
            if self.current is not None and self.xyz is not None:
                self.on_goal(String(data=json.dumps(self.current)))   # resume

    def on_result(self, fut):
        if self.held:
            return                                   # cancelled on purpose
        ok = fut.result().status == GoalStatus.STATUS_SUCCEEDED
        g = self.current
        if ok and g['kind'] == 'deliver':
            # arrived: hand over to the bedside verifier before marking done
            self.verify_req.publish(String(data=json.dumps(
                {'order': g['order'], 'bed': g['bed']})))
        else:
            self.report('done' if ok else 'failed', g['order'])

    def on_verify(self, m):
        r = json.loads(m.data)          # {"order": id, "ok": bool}
        self.report('done' if r['ok'] else 'failed', r['order'])

    def on_stop(self, m):
        if m.data and self.handle is not None:
            self.handle.cancel_goal_async()

    def report(self, state, order):
        self.status.publish(String(data=json.dumps({'state': state, 'order': order})))


def main():
    rclpy.init()
    rclpy.spin(NavExecutor())
    rclpy.shutdown()
