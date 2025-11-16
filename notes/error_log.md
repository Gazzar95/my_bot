# Error Log

- Add each issue as a new section so the history stays organized.
- Include the full error text, root cause, and the steps you used to resolve it.

## No parameter file provided (gazebo_ros2_control)
- **Error**: `[gazebo_ros2_control]: No parameter file provided. Configuration might be wrong`
- **Cause**: `<parameters>` pointed to an old workspace path (`/home/omar/dev_ws/...`) so Gazebo could not load `my_controller.yaml`.
- **Fix**: Update `ros2_control.xacro` to use `$(find my_bot)/config/my_controller.yaml`, then rerun `colcon build` and source `install/setup.bash`.

## gzclient camera assertion
- **Error**: `gzclient: ... boost::shared_ptr<gazebo::rendering::Camera>::operator->() ... Assertion 'px != 0' failed`
- **Status**: Still investigating. Workaround: launch Gazebo headless or disable the camera GUI plugin until the sensor config is fixed.
