# Error Log

- Add each issue as a new section so the history stays organized.
- Include the full error text, root cause, and the steps you used to resolve it.

## 1. No parameter file provided (gazebo_ros2_control)
- **Error**: `[gazebo_ros2_control]: No parameter file provided. Configuration might be wrong`
- **Cause**: `<parameters>` either pointed to an old workspace path (`/home/omar/dev_ws/...`) or was nested under the wrong XML tag, so Gazebo could not resolve `my_controller.yaml`.
- **Fix**: Update `ros2_control.xacro` so the `<plugin name="gazebo_ros2_control">` block directly contains `<parameters>$(find my_bot)/config/my_controller.yaml</parameters>` (and rebuild + source) to let Gazebo locate the YAML regardless of workspace path.

## 2. gzclient camera assertion
- **Error**: `gzclient: ... boost::shared_ptr<gazebo::rendering::Camera>::operator->() ... Assertion 'px != 0' failed`
- **Status**: Still investigating. Workaround: launch Gazebo headless or disable the camera GUI plugin until the sensor config is fixed.

## 3. parser error Couldn't parse parameter override rule
- **Error**: `[gazebo_ros2_control]: parser error Couldn't parse parameter override rule: '--param robot_description:=<?xml ...'`
- **Cause**: `gazebo_ros2_control` spawns an internal ROS 2 node with `--param robot_description:=<URDF>` and the raw XML (angle brackets, commas, quotes) is blowing up the `rcl` argument parser.
- **Status**: Investigating. Need to keep plugin from passing the full URDF on the command-line—either load the URDF from a file or supply it via a params file/launch argument so the plugin never has to parse the raw XML string.
