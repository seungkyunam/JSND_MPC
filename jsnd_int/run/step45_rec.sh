#!/usr/bin/env bash
# Step 45 comparison recorder (tmux "rec"). Run AFTER step45_run.sh has started.
# usage: bash ~/JSND_INT/run/step45_rec.sh <pp|mpc|improved>
# stop : tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec
# Records: compute time (mpc_elapsed/cycle/loop_period), solver status, target speed+steer
# (/vesc/high_level/ackermann_cmd = controller, /vesc/ackermann_cmd = after mux),
# actual speed (/car_state/odom, /vesc/sensors/core ERPM), pose/EKF/IMU, mux/estop, /scan.
set -eo pipefail
M=${1:?mode pp|mpc|improved}
case "$M" in pp|mpc|improved) ;; *) echo 'mode must be pp, mpc or improved'; exit 2;; esac
pgrep -f '[v]esc_driver_node' >/dev/null || { echo "ABORT: vehicle stack not running"; exit 1; }
tmux has-session -t rec 2>/dev/null && { echo "ABORT: tmux 'rec' exists"; exit 1; }
H=$HOME
mkdir -p "$H/bench_logs/bags"
OUT=$H/bench_logs/bags/step45_${M}_$(date +%Y%m%d_%H%M%S)
T="/car_state/odom /car_state/odom_frenet /pf/pose/odom /vesc/sensors/imu_corrected /imu/data \
/vesc/high_level/ackermann_cmd /vesc/ackermann_cmd /vesc/sensors/core /state_machine \
/simple_mux/active_host /simple_mux/estop_state /scan \
/controller/mpc_status /controller/mpc_elapsed_ms /controller/mpc_cycle_ms /controller/mpc_loop_period_ms \
/controller/mpc_diagnostics /tracking/obstacles /local_waypoints /global_waypoints \
/behavior_strategy /state_machine/debug /state_machine/trailing_target"
tmux new-session -d -s rec "bash -c 'source /opt/ros/jazzy/setup.bash; source $H/JSND/install/local_setup.bash; \
source $H/JSND_INT/install/local_setup.bash; \
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$H/JSND/src/racing_stack/cyclonedds.xml; \
ros2 bag record -s mcap -o $OUT $T; exec bash'"
echo "recording -> $OUT"
echo "stop  : tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec"
