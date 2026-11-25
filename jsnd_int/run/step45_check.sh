#!/usr/bin/env bash
# Step 45 read-only check of the running stack (no motor/estop action).
# usage: bash ~/JSND_INT/run/step45_check.sh <pp|mpc|improved>
M=${1:?mode}
source /opt/ros/jazzy/setup.bash; source "$HOME/JSND/install/local_setup.bash"; source "$HOME/JSND_INT/install/local_setup.bash"
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$HOME/JSND/src/racing_stack/cyclonedds.xml"
if [[ "$M" != pp ]]; then
  for p in mpc_heading_robust mpc_history_prediction mpc_speed_source mpc_horizon control_loop_rate_hz mpc_latency_comp_s; do
    echo -n "$p: "; ros2 param get /controller_manager $p 2>&1 | tail -1; done
fi
echo -n "ekf imu0: "; ros2 param get /ekf_localization imu0 2>&1 | tail -1
for t in /car_state/odom /vesc/sensors/imu_corrected /vesc/high_level/ackermann_cmd; do
  echo "== hz $t"; timeout 4 ros2 topic hz $t 2>&1 | grep -m1 'average rate' || echo "  (no data)"; done
echo "== /car_state/odom twist.angular.z (EKF yaw rate, used by heading_robust)"
timeout 3 ros2 topic echo --once --field twist.twist.angular.z /car_state/odom 2>&1 | head -2
grep -h '\[Step 45\]' $(ls -t "$HOME"/bench_logs/step45_${M}_*.log 2>/dev/null | head -1) 2>/dev/null | head -1
