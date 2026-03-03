# Common SLAM sim commands

Command 1 — Teleop (keyboard) into twist_mux keyboard input:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

Command 2 — Launch SLAM Toolbox with your async mapper params and sim time:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/omar/projects/dev_ws/src/my_bot/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

Command 3 — Launch Nav2 and route velocity commands through twist_mux:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  --ros-args -r /cmd_vel:=/cmd_vel_nav
```
