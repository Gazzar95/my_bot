# Common SLAM sim commands

Command 1 — Teleop (keyboard) remapped to the diff drive controller:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Command 2 — Launch SLAM Toolbox with your async mapper params and sim time:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/omar/projects/dev_ws/src/my_bot/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```
