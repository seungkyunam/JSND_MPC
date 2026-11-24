#!/usr/bin/env bash
# Speed-ramp recorder (step 19): records the topics speed_report.py needs, in tmux "rec".
# usage: bash ~/JSND_INT/run/speed_rec.sh <2p25|3p0|4p0|5p0|6p0|7p0>   stop: tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec
set -eo pipefail
P=${1:?profile}; INT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd); H=$(dirname "$INT")
tmux has-session -t race 2>/dev/null || { echo "ABORT: race stack not running"; exit 1; }
tmux has-session -t rec 2>/dev/null && { echo "ABORT: tmux 'rec' exists"; exit 1; }
OUT=$H/bench_logs/bags/${P}_$(date +%Y%m%d_%H%M%S)
T="/state_machine /pf/pose/odom /vesc/sensors/imu_corrected /car_state/odom /car_state/odom_frenet /vesc/ackermann_cmd /vesc/high_level/ackermann_cmd /vesc/sensors/core \
/simple_mux/active_host /simple_mux/estop_state /controller/mpc_status /controller/mpc_elapsed_ms"
tmux new-session -d -s rec "bash -c 'source /opt/ros/jazzy/setup.bash; source $H/JSND/install/local_setup.bash; \
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$H/JSND/src/racing_stack/cyclonedds.xml; \
ros2 bag record -s mcap -o $OUT $T; exec bash'"
echo "recording -> $OUT"; echo "stop  : tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec"; echo "report: python3 $INT/run/speed_report.py $OUT"
