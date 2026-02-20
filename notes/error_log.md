# Error Log

- Add each issue as a new section so the history stays organized.
- For each issue, capture error text, suspected cause, and a troubleshooting log of actions + results.
- Keep troubleshooting steps chronological so it's easy to see what was tried.

## 1. No parameter file provided (gazebo_ros2_control)
- **Status**: Resolved
- **Error**: `[gazebo_ros2_control]: No parameter file provided. Configuration might be wrong`
- **Suspected cause**: `<parameters>` pointed to an old workspace path (`/home/omar/dev_ws/...`) or was nested under the wrong XML tag, so Gazebo could not resolve `my_controller.yaml`.
- **Troubleshooting log**:
  - **Action**: Updated `ros2_control.xacro` so the `<plugin name="gazebo_ros2_control">` block directly contains `<parameters>$(find my_bot)/config/my_controller.yaml</parameters>`.
  - **Result**: Gazebo locates the YAML regardless of workspace path and the error stops.
- **Resolution**: Rebuild + source after the xacro change.

## 2. gzclient camera assertion
- **Status**: Resolved
- **Error**: `gzclient: ... boost::shared_ptr<gazebo::rendering::Camera>::operator->() ... Assertion 'px != 0' failed`
- **Suspected cause**: Gazebo plugins (camera/depth camera) weren’t being found because Gazebo’s environment wasn’t sourced; the GUI tried to dereference a camera that never initialized and crashed.
- **Troubleshooting log**:
  - **Action**: Sourced Gazebo after the workspace: `source /home/omar/projects/dev_ws/install/setup.bash && source /usr/share/gazebo/setup.sh`.
  - **Result**: Camera plugins load and the GUI no longer crashes.
- **Resolution**: Keep Gazebo sourced after the workspace.

## 3. parser error Couldn't parse parameter override rule
- **Status**: Resolved
- **Error**: `[gazebo_ros2_control]: parser error Couldn't parse parameter override rule: '--param robot_description:=<?xml ...'`
- **Suspected cause**: `gazebo_ros2_control` pushes the URDF onto its node argv (`--param robot_description:=<URDF>`), and colon/URL content inside XML comments made `rcl`'s argument parser fail.
- **Troubleshooting log**:
  - **Action**: Removed/adjusted problematic comments (colon/URL/`###` text) from the xacros.
  - **Result**: Controller manager loads cleanly after rebuild + source.
- **Resolution**: Keep URDF comments parser-safe.

## 4. Raspberry Pi network-layer issue (MAC/ARP conflict)
- **Status**: Resolved
- **Error**: Intermittent SSH failures; router shows Pi online but unreachable.
- **Suspected cause**: Pi previously connected via Ethernet, then switched to Wi-Fi; same IP (`192.168.8.143`) reused with different MACs, poisoning router + laptop ARP caches.
- **Troubleshooting log**:
  - **Action**: Observed ping flapping, "No route to host", and "Destination Host Unreachable" while router showed the Pi online.
  - **Result**: Symptoms consistent with ARP cache conflict.
  - **Action**: Power cycled network interfaces and issued a new IP/DHCP reservation.
  - **Result**: ARP cache stabilized and network reachability returned.
- **Resolution**: Avoid IP reuse across NICs; keep a DHCP reservation per MAC.

## 5. Raspberry Pi power integrity issue (undervoltage)
- **Status**: Open
- **Error**: Wi-Fi drops and peripherals intermittently disappear.
- **Suspected cause**: Laptop USB power cannot sustain Pi + Wi-Fi + USB load; voltage sag below ~4.63 V.
- **Troubleshooting log**:
  - **Action**: Disconnected the USB hub (lidar + Arduino) from the Pi.
  - **Result**: Issue persists; hub load may not be the sole cause.
  - **Action**: Powered the Pi from a separate power source (not the USB hub).
  - **Result**: Used laptop power; after ~20 minutes the connection stabilized. Measured 5.2 V at the Pi with a multimeter.
  - **Action**: Ran `dmesg -w` while powered from the laptop.
  - **Result**: Saw repeated "Undervoltage detected" messages.
  - **Action**: Plan to power from battery and observe stability.
  - **Result**: Pending.
- **Next steps**: Power from the robot battery with proper regulation and confirm that undervoltage messages stop and Wi-Fi/USB devices stay stable.

## 6. RPLidar A1M8 scan mode mismatch
- **Status**: Resolved
- **Error**: RPLidar scan would not start at 115200/256000 baud despite device enumeration and healthy status.
- **Suspected cause**: Driver attempted an unsupported scan mode on A1M8 firmware.
- **Troubleshooting log**:
  - **Action**: Verified USB enumeration and permissions (`/dev/ttyUSB0`, `/dev/serial/by-id`, dialout).
  - **Result**: Device enumerated correctly with expected permissions.
  - **Action**: Confirmed driver reads SDK version, firmware, hardware rev, and health = 0.
  - **Result**: No hardware faults reported.
  - **Action**: Tried baudrates (115200, 256000).
  - **Result**: No change; scan still failed to start.
  - **Action**: Set explicit scan mode: `'scan_mode': 'Standard'`.
  - **Result**: Scan starts successfully.
- **Resolution**: Force simple scan mode and keep A1M8 baudrate:
  ```python
  'serial_baudrate': 115200,
  'scan_mode': 'Standard',
  ```