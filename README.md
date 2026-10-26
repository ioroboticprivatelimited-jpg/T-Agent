# T-Agent: priority- and crowd-aware task decisions for ROS 2

T-Agent is a small decision layer that sits above Nav2. It chooses which task a service robot
should serve next and changes its behaviour when the route ahead gets crowded. It was built for a
hospital food-delivery robot, but nothing in it is specific to hospitals.

> **Status:** early research code. The syntax is checked and the decision policy was tested with
> stubbed ROS. It has **not yet been run on Nav2 or a real robot** from this repository. Thresholds
> are placeholders that need tuning on your robot.

## What it does

- **Ranks queued tasks** with a score: `S = w_p·priority + w_t·waiting − w_d·distance − w_c·crowd`
- **Battery rule:** goes to the dock when charge drops below a floor.
- **Reacts to crowds:** low crowd continues, medium slows down, high replans (urgent task) or holds
  (other tasks), with hysteresis so it does not flip back and forth.
- **Independent safety stop:** a separate node latches a stop on a close obstacle, a silent LiDAR or
  an e-stop, and only a person can reset it.
- **Logs every trip to CSV** (trip time, worst crowd value, holds, replans, sensing-to-command delay)
  so you can analyse it in Excel.

## Package layout

```
hospital_delivery/
├── package.xml  setup.py  README.md
├── config/agent.yaml           weights, thresholds, bed poses, safety limits
├── launch/agent.launch.py      starts all nodes with agent.yaml
└── hospital_delivery/
    ├── order_manager.py        /orders/raw -> /orders/new
    ├── t_agent.py              scoring, crowd reaction, trip log
    ├── crowd_monitor.py        LiDAR -> /crowd/level
    ├── nav_executor.py         /nav/goal -> Nav2 NavigateToPose, hold and resume
    ├── safety_supervisor.py    latched /safety/stop
    ├── bedside_verifier.py     confirms the delivery before closing the order
    └── task_planner.py         earlier planner without the crowd term (for comparison)
```

## Requirements

- ROS 2 (tested design target: Humble or newer) with Nav2 running (`navigate_to_pose` action)
- Python 3, `rclpy`, `nav2_msgs`, `sensor_msgs`, `std_msgs`
- Your robot must provide: `/scan` (LaserScan), `/battery` (Float32, 0 to 1), `/robot/pose2d`
  (String, JSON `[x, y]`), and the Nav2 speed filter listening on `/speed_limit`

## Build and run

```bash
cd ~/ros2_ws/src
# copy or clone this package here
cd ~/ros2_ws
colcon build --packages-select hospital_delivery
source install/setup.bash
ros2 launch hospital_delivery agent.launch.py
```

Send a test order:

```bash
ros2 topic pub --once /orders/raw std_msgs/String \
  '{data: "{\"bed\": \"bed_2\", \"urgency\": 0.8, \"deadline_s\": 1800}"}'
```

## Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/orders/raw` | std_msgs/String (JSON) | in | New orders from a dashboard or script |
| `/orders/new` | std_msgs/String (JSON) | order_manager → t_agent | Validated orders |
| `/crowd/level` | std_msgs/String (JSON) | crowd_monitor → t_agent | Crowd value and scan time |
| `/nav/goal` | std_msgs/String (JSON) | t_agent → nav_executor | Deliver or dock goal |
| `/nav/hold` | std_msgs/Bool | t_agent → nav_executor | Pause and resume the current goal |
| `/speed_limit` | nav2_msgs/SpeedLimit | t_agent → Nav2 | Percentage speed limit |
| `/nav/status` | std_msgs/String (JSON) | nav_executor → t_agent | active, done or failed |
| `/safety/stop` | std_msgs/Bool | safety_supervisor → nav_executor | Latched stop |
| `/verify/request`, `/verify/result`, `/verify/confirm` | std_msgs/String (JSON) | verifier loop | Delivery confirmation |

## Tuning (`config/agent.yaml`)

| Parameter | Meaning |
|---|---|
| `w_priority`, `w_time`, `w_distance`, `w_crowd` | Score weights |
| `th_mid`, `th_high`, `hyst` | Crowd levels at which it slows down, reroutes or holds, and the margin before it returns |
| `mid_pct`, `high_pct` | Speed limits (percent) at medium and high crowd |
| `urgent` | Urgency at or above which a task replans instead of holding |
| `battery_floor` | Charge fraction that sends the robot to the dock |
| `range_m`, `sector_deg` (crowd_monitor) | How near and how wide the crowd value looks |
| `min_range_m`, `scan_timeout_s` (safety_supervisor) | Stop distance and LiDAR timeout |

Bed poses in `agent.yaml` are placeholders. Replace them with coordinates from your own map.

Note: the crowd value counts any close object in front of the robot (people, trolleys, walls), not
only people. Tune `range_m` and `sector_deg` for your robot, or replace it with a people detector.

## Trip log

Each finished trip is appended to `~/t_agent_trips.csv`:

`time, order, bed, urgency, result, trip_s, crowd_max, holds, replans, delay_max_s`

## Results so far

Measured on a delivery robot (not yet including crowd tests):

| Setting | Delivery success | Collisions |
|---|---|---|
| Simulation | 95% | 0 |
| Lab corridor | 90% | 0 |
| Ward mock-up | 75% | 0 |

Crowd-density tests and a Nav2-only baseline comparison: _to be added._

## Limitations and next steps

- Not yet validated on Nav2 or hardware from this repository.
- The decision logic is rule-based. Learned crowd prediction or natural-language orders are future work.
- The reroute step re-sends the goal to force a replan. A custom Nav2 cost layer for crowds would be cleaner.

## Citation

[Add your paper or talk reference here.]

## License

[Choose a license, for example Apache-2.0 or MIT, and add a LICENSE file. `package.xml` currently says Apache-2.0.]

## Author

R Arun Balaji
