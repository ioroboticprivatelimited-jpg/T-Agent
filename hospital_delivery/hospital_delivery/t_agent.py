"""T-Agent: priority- and crowd-aware task decisions above Nav2.

Dispatch (1 Hz):   S_i = w_p*P_i + w_t*T_i - w_d*D_i - w_c*C
                   battery below floor -> dock.
Reaction (10 Hz):  crowd level LOW / MID / HIGH with hysteresis.
                   LOW  : full speed
                   MID  : speed limit mid_pct %
                   HIGH : high-priority task -> replan + slow (rate limited)
                          other tasks        -> hold until the crowd eases
Every finished trip is appended to a CSV (open it in Excel) with the worst
crowd value, number of holds/replans and the sensing-to-command delay.
"""
import csv
import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Bool, Float32, String

LOW, MID, HIGH = 0, 1, 2


class TAgent(Node):
    def __init__(self):
        super().__init__('t_agent')
        for k, v in dict(w_priority=0.4, w_time=0.25, w_distance=0.15, w_crowd=0.2,
                         battery_floor=0.20, th_mid=0.25, th_high=0.55, hyst=0.08,
                         mid_pct=60.0, high_pct=40.0, urgent=0.7,
                         replan_period_s=5.0,
                         log_path=os.path.expanduser('~/t_agent_trips.csv')).items():
            self.declare_parameter(k, v)
        self.queue, self.beds = [], {}
        self.pose, self.battery = (0.0, 0.0), 1.0
        self.busy, self.cur = False, None
        self.level, self.c = LOW, 0.0
        self.held, self.last_replan = False, 0.0
        self.stats = {}
        self.goal_pub = self.create_publisher(String, '/nav/goal', 10)
        self.hold_pub = self.create_publisher(Bool, '/nav/hold', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.create_subscription(String, '/orders/new', lambda m: self.queue.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/nav/beds', self.on_beds, 10)
        self.create_subscription(String, '/nav/status', self.on_status, 10)
        self.create_subscription(String, '/crowd/level', self.on_crowd, 10)
        self.create_subscription(String, '/robot/pose2d', lambda m: setattr(self, 'pose', tuple(json.loads(m.data))), 10)
        self.create_subscription(Float32, '/battery', lambda m: setattr(self, 'battery', m.data), 10)
        self.create_timer(1.0, self.dispatch_tick)
        self._ensure_log()

    # ---- inputs
    def on_beds(self, m):
        self.beds = {k: tuple(v) for k, v in json.loads(m.data).items()}

    def on_status(self, m):
        s = json.loads(m.data)
        if s['state'] in ('done', 'failed') and self.cur:
            self.log_trip(s['state'])
            self.busy, self.cur = False, None
            self.set_hold(False)
        elif s['state'] == 'idle':
            self.busy = False

    def on_crowd(self, m):
        d = json.loads(m.data)
        self.c = d['c']
        self.react()
        # sensing-to-command delay, measured at the moment the command is sent
        lat = self.get_clock().now().nanoseconds * 1e-9 - d['stamp']
        if self.cur:
            self.stats['lat_max'] = max(self.stats.get('lat_max', 0.0), lat)
            self.stats['c_max'] = max(self.stats.get('c_max', 0.0), self.c)

    # ---- decisions
    def score(self, o):
        p = lambda n: self.get_parameter(n).value
        t = min(1.0, (time.time() - o['created']) / max(o['deadline'], 1.0))
        bx, by = self.beds.get(o['bed'], self.pose)
        d = min(1.0, math.hypot(bx - self.pose[0], by - self.pose[1]) / 30.0)
        return (p('w_priority') * o['urgency'] + p('w_time') * t
                - p('w_distance') * d - p('w_crowd') * self.c)

    def dispatch_tick(self):
        if self.busy:
            return
        if self.battery < self.get_parameter('battery_floor').value:
            self.start({'kind': 'dock', 'order': 0, 'urgency': 0.0})
        elif self.queue:
            best = max(self.queue, key=self.score)
            self.queue.remove(best)
            self.start({'kind': 'deliver', 'order': best['id'], 'bed': best['bed'],
                        'urgency': best['urgency']})

    def start(self, g):
        self.busy, self.cur = True, g
        self.stats = {'t0': time.time(), 'holds': 0, 'replans': 0}
        self.goal_pub.publish(String(data=json.dumps(g)))
        self.get_logger().info(f'dispatch {g}')

    def react(self):
        p = lambda n: self.get_parameter(n).value
        mid, high, h = p('th_mid'), p('th_high'), p('hyst')
        lv = self.level
        if self.c >= high or (lv == HIGH and self.c > high - h):
            lv = HIGH
        elif self.c >= mid or (lv >= MID and self.c > mid - h):
            lv = MID
        else:
            lv = LOW
        self.level = lv
        if not self.cur:
            return
        urgent = self.cur.get('urgency', 0.0) >= p('urgent')
        if lv == LOW:
            self.set_speed(100.0); self.set_hold(False)
        elif lv == MID:
            self.set_speed(p('mid_pct')); self.set_hold(False)
        else:
            self.set_speed(p('high_pct'))
            if urgent:
                self.set_hold(False)
                if time.time() - self.last_replan > p('replan_period_s'):
                    self.last_replan = time.time()
                    self.stats['replans'] = self.stats.get('replans', 0) + 1
                    self.goal_pub.publish(String(data=json.dumps(self.cur)))  # forces a replan
            else:
                self.set_hold(True)

    def set_speed(self, pct):
        m = SpeedLimit(); m.percentage = True; m.speed_limit = float(pct)
        self.speed_pub.publish(m)

    def set_hold(self, on):
        if on != self.held:
            self.held = on
            if on and self.stats:
                self.stats['holds'] = self.stats.get('holds', 0) + 1
            self.hold_pub.publish(Bool(data=on))

    # ---- trip log (open in Excel)
    def _ensure_log(self):
        path = self.get_parameter('log_path').value
        if not os.path.exists(path):
            with open(path, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'order', 'bed', 'urgency', 'result',
                                        'trip_s', 'crowd_max', 'holds', 'replans', 'delay_max_s'])

    def log_trip(self, result):
        s, g = self.stats, self.cur
        with open(self.get_parameter('log_path').value, 'a', newline='') as f:
            csv.writer(f).writerow([time.strftime('%Y-%m-%d %H:%M:%S'), g['order'],
                g.get('bed', 'dock'), g.get('urgency', 0), result,
                round(time.time() - s.get('t0', time.time()), 1),
                round(s.get('c_max', 0), 3), s.get('holds', 0), s.get('replans', 0),
                round(s.get('lat_max', 0), 3)])


def main():
    rclpy.init()
    rclpy.spin(TAgent())
    rclpy.shutdown()
