#!/usr/bin/env bash
# Stop all processes of a forklift simulation/bringup (Gazebo, bridge, ROS nodes).
# Use this script after an interrupted launch. Remaining processes can send duplicate data.
patterns=(
  'ros2 launch forklift_'
  'gz sim'
  'ros_gz_bridge/parameter_bridge'
  'ros_gz_sim/create'
  'robot_state_publisher/robot_state_publisher'
  'controller_manager/ros2_control_node'
  'controller_manager/spawner'
  'twist_mux/twist_mux'
  'forklift_control/lift_controller'
  'robot_localization/ekf_node'
  'rviz2/rviz2'
  'install/forklift_'      # all nodes of this workspace (load_handler, mission_manager, fleet_*, ...)
)
# Fast DDS leaves shared-memory segments of killed processes in /dev/shm (hundreds after many
# interrupted runs). New nodes then try dead endpoints: lost service responses, goals without
# acknowledgement. Delete only the segments that no process has open.
cleanup_shm() {
  for f in /dev/shm/fastrtps_*; do
    [[ -e "$f" ]] && ! fuser -s "$f" 2>/dev/null && rm -f "$f"
  done
}
for p in "${patterns[@]}"; do pkill -INT -f "$p" 2>/dev/null; done
for _ in $(seq 1 50); do
  alive=0
  for p in "${patterns[@]}"; do pgrep -f "$p" >/dev/null && alive=1; done
  [[ $alive -eq 0 ]] && { cleanup_shm; echo "simulation stopped"; exit 0; }
  sleep 0.2
done
for p in "${patterns[@]}"; do pkill -KILL -f "$p" 2>/dev/null; done
cleanup_shm
echo "simulation killed (did not exit on SIGINT)"
