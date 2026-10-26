# hospital_delivery: T-Agent for ROS 2

Nodes: order_manager, t_agent (priority + crowd decisions), crowd_monitor (LiDAR crowd value),
nav_executor (Nav2 goals, hold/resume), safety_supervisor (latched stop), bedside_verifier.
Older task_planner.py is kept for comparison (it has no crowd term).

Build:   colcon build --packages-select hospital_delivery && source install/setup.bash
Run:     ros2 launch hospital_delivery agent.launch.py
Order:   ros2 topic pub --once /orders/raw std_msgs/String '{data: "{\"bed\":\"bed_2\",\"urgency\":0.8}"}'
Log:     ~/t_agent_trips.csv  (open in Excel: trip_s, crowd_max, holds, replans, delay_max_s)

Needs from your robot: /scan, /battery (Float32), /robot/pose2d (JSON [x, y]), Nav2 with
navigate_to_pose, and the Nav2 speed filter listening on /speed_limit.
Status: syntax-checked and the decision policy tested with stubbed ROS. Not yet run on Nav2.
