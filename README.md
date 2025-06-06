# my_bot – ROS 2 Learning & Prototyping Robot

🤖 A foundational ROS 2 project built as part of my transition into robotics. Currently supports differential-drive simulation, URDF modeling, visualization, and teleop control. Will serve as scaffolding for a more advanced mobile manipulator project in 2025–2026.

## Project Scope

This package is a personal learning tool to build practical skills in ROS 2 development, simulation, and integration. Over time, it will evolve into a full-stack mobile manipulator prototype incorporating navigation, manipulation, and perception pipelines.

✅ Implemented so far:
- Differential-drive robot in URDF/Xacro
- Working TF tree (`base_link`, `odom`, wheels)
- RViz 2 visualization
- Gazebo simulation with physics and plugins
- Keyboard teleop using `teleop_twist_keyboard`
- Parameter tuning via YAML config
- ROS 2 launch system

🧭 Roadmap (future additions):
- Add robot arm (URDF + moveit2)
- Integrate Nav2 for mapping and localization
- Add camera or LiDAR sensor model
- SLAM integration and autonomous behavior
- Mobile Manipulator Capstone (Modern Robotics ROS port)

## Setup Instructions

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/Gazzar95/my_bot.git

# Build and source
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch robot in RViz and Gazebo
ros2 launch my_bot bringup.launch.py
