# AMR Robot Project Memory

## Hardware
- Robot: Custom AMR with hoverboard wheels, ESP32 (USB1), Arduino Mega (ACM0), RPLidar A1 (USB0), IMU
- Pi: 100.78.53.65, user pi, passwordless docker sudo
- Docker container: `amr` (--restart unless-stopped policy)
- Workspace bind-mounted: /home/pi/amr_ws → /root/amr_ws

## Connectivity from laptop
- export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
- export ROS_DOMAIN_ID=0
- export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>100.113.124.97</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="100.78.53.65"/></Peers></Discovery></Domain></CycloneDDS>'

## CycloneDDS on Pi (CRITICAL)
- Container env has AllowMulticast=false and Peer=100.113.124.97 (laptop) only
- This breaks intra-Pi node discovery — must override at launch with docker exec -e
- All 8 node launches MUST use: docker exec -e CYCLONEDDS_URI="<see below>" amr bash -c ...
- Pi CYCLONEDDS_URI (adds self as peer for local discovery):
  <CycloneDDS><Domain><General><NetworkInterfaceAddress>100.78.53.65</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="100.78.53.65"/><Peer address="100.113.124.97"/></Peers></Discovery></Domain></CycloneDDS>
- pyserial must be installed in container: docker exec amr apt-get install -y python3-serial

## Critical configs (all in /home/pi/amr_ws/ on Pi)
- ekf.yaml — fixed (odom0/imu0 not odom0.0/imu0.0)
- amcl_params.yaml — base_frame_id: base_link (NOT base_footprint)
- nav2_params.yaml — includes collision_monitor config
- slam_params.yaml — mode: localization
- lidar_filter.yaml — ±115° (not yet active)
- my_map.yaml + my_map.pgm — saved map (192x230)

## Code fixes applied
- serial_bridge.py: 172× multiplier on rpm (EMPIRICALLY needed, do not remove)
- serial_bridge.py: RPM_DEADZONE = 5.0
- safety_monitor.py: debounce (3 STOP + 2 CLEAR within 0.35s)

## Launch order required
1. rplidar_ros: `ros2 launch rplidar_ros rplidar.launch.py` (serial_port=/dev/ttyUSB0 hardcoded)
2. serial_bridge: `ros2 run serial_bridge serial_bridge --ros-args -p publish_tf:=false`
3. safety_monitor: `ros2 run serial_bridge safety_monitor`
4. robot_state_publisher: `python3 /root/amr_ws/start_rsp.py`
5. joint_state_publisher: `ros2 run joint_state_publisher joint_state_publisher`
6. ekf_node: `ros2 run robot_localization ekf_node --ros-args --params-file /root/amr_ws/ekf.yaml`
7. AMCL: `ros2 launch nav2_bringup localization_launch.py params_file:=/root/amr_ws/amcl_params.yaml map:=/root/amr_ws/my_map.yaml`
8. Nav2: `ros2 launch /root/amr_ws/navigation_launch.py params_file:=/root/amr_ws/nav2_params.yaml`
   NOTE: Uses CUSTOM launch file at /root/amr_ws/navigation_launch.py (NOT nav2_bringup default)
   Reason: Jazzy's navigation_launch.py hardcodes docking_server in lifecycle_nodes which
   crashes without dock hardware. Custom file removes docking_server from lifecycle.

## Known issues
- LiDAR 230° FOV: filter installed but not active (waiting for RViz verification)
- back ultrasonic sensor noisy (debounce handles it)
- Container's container-only /tmp files are lost on restart - use /root/amr_ws/ paths

## Current status
- All nodes running and lifecycle ACTIVE (2026-06-06, after container recreation)
- TF tree complete: map → odom → base_link → laser
- Nav2 lifecycle activates fully (27 active nodes)
- CycloneDDS cross-machine comms working (Pi ↔ laptop at 100.113.124.97)
- Last left off at: about to set 2D Pose Estimate in RViz, then test 2D Goal Pose

## Rules for Claude Code
- DO NOT remove the 172× multiplier in serial_bridge.py — empirically calibrated
- DO NOT modify Arduino firmware
- DO NOT auto-restart healthy nodes
- Ask before destructive changes
- Save state to /home/pi/amr_ws/STATUS.md regularly
