#!/usr/bin/env bash
# Step 45 read-only check of the running stack (no motor/estop action).
# usage: bash ~/JSND_INT/run/step45_check.sh <pp|mpc>
M=${1:?mode}
case "$M" in pp|mpc) ;; *) echo 'mode must be pp or mpc'; exit 2;; esac
source /opt/ros/jazzy/setup.bash; source "$HOME/JSND/install/local_setup.bash"
vehicle_overlay=${JSND_INT_OVERLAY:-$HOME/JSND_INT}; source "$vehicle_overlay/install/local_setup.bash"
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$HOME/JSND/src/racing_stack/cyclonedds.xml"
failed=0
if [[ "$M" == mpc ]]; then
  for p in mpc_heading_robust mpc_history_prediction mpc_speed_source mpc_horizon mpc_preview_dt \
           mpc_speed_response_tau_s mpc_q_position mpc_q_heading mpc_q_speed mpc_r_accel \
           mpc_deadline_s mpc_solver_time_limit_s control_loop_rate_hz mpc_latency_comp_s; do
    echo -n "$p: "; ros2 param get /controller_manager $p 2>&1 | tail -1; done
  expect() {
    actual=$(timeout 5 ros2 param get /controller_manager "$1" 2>/dev/null | sed -n 's/.*value is: //p' | tail -1)
    [[ "$actual" == "$2" ]] || { echo "FAIL $1=$actual expected $2"; failed=1; }
  }
  expect control_loop_rate_hz 50
  expect mpc_speed_source integrated
  expect mpc_horizon 10
  expect mpc_preview_dt 0.10888888888888888
  expect mpc_speed_response_tau_s 0.12
  expect mpc_history_prediction true
  expect mpc_heading_robust false
fi
echo -n "ekf imu0: "; ros2 param get /ekf_localization imu0 2>&1 | tail -1
for t in /car_state/odom /vesc/sensors/imu_corrected /vesc/high_level/ackermann_cmd; do
  echo "== hz $t"; timeout 4 ros2 topic hz $t 2>&1 | grep -m1 'average rate' || echo "  (no data)"; done
echo "== /car_state/odom twist.angular.z (EKF yaw rate, used by heading_robust)"
timeout 3 ros2 topic echo --once --field twist.twist.angular.z /car_state/odom 2>&1 | head -2
grep -h '\[Step 45\]' $(ls -t "$HOME"/bench_logs/step45_${M}_*.log 2>/dev/null | head -1) 2>/dev/null | head -1
if (( failed )); then echo 'FINAL CONFIG CHECK: FAILED'; exit 1; fi
echo 'FINAL CONFIG CHECK: PASS (read-only; no command was sent)'
