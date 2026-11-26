#!/usr/bin/env bash
# PP vs MPC comparison recorder (tmux "rec"). Run AFTER the vehicle stack has started.
# usage: bash ~/JSND_INT/compare_pp_mpc/cmp_rec.sh <pp|mpc> <cap_label> <layout_id> [note]
#   cap_label : speed-cap condition name, e.g. cap1p0 / cap5p0 (must match between PP and MPC runs)
#   layout_id : obstacle layout id from layouts.yaml, or "none"
# stop : tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec
# Superset of step45_rec.sh topics + /lap_data (lap_analyser) + /controller/latency + /vesc/odom.
# Writes <bag>/../<bag>.meta.json with the run conditions (read by cmp_analyze.py).
# Read-only w.r.t. the stack: only ros2 bag record + one `ros2 param get` per controller.
set -eo pipefail
C=${1:?controller pp|mpc}; CAP=${2:?cap label e.g. cap1p0}; LAYOUT=${3:?layout id or none}; NOTE=${4:-}
case "$C" in pp|mpc) ;; *) echo 'controller must be pp or mpc'; exit 2;; esac
[[ "$CAP" =~ ^[a-zA-Z0-9_]+$ && "$LAYOUT" =~ ^[a-zA-Z0-9_]+$ ]] || { echo 'bad cap/layout label'; exit 2; }
pgrep -f '[v]esc_driver_node' >/dev/null || { echo "ABORT: vehicle stack not running"; exit 1; }
tmux has-session -t rec 2>/dev/null && { echo "ABORT: tmux 'rec' exists"; exit 1; }
H=$HOME
source /opt/ros/jazzy/setup.bash; source $H/JSND/install/local_setup.bash; source $H/JSND_INT/install/local_setup.bash
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$H/JSND/src/racing_stack/cyclonedds.xml
mkdir -p "$H/bench_logs/bags"
TS=$(date +%Y%m%d_%H%M%S)
OUT=$H/bench_logs/bags/cmp_${C}_${CAP}_${LAYOUT}_${TS}
# Conditions actually active in the running stack (recorded, not assumed).
latest_log=$(ls -t $H/bench_logs/step45_*.log 2>/dev/null | head -1 || true)
pget() { timeout 5 ros2 param get "$1" "$2" 2>/dev/null | sed -n 's/.*value is: //p' | head -1; }
mpc_cap=$(pget /controller_manager mpc_max_speed_mps)
phase=$(pget /speed_sector_tuner phase_multiplier)
sector=$(pget /speed_sector_tuner sector_scaling)
default_scaling=$(pget /speed_sector_tuner default_scaling)
cat > "$OUT.meta.json" <<EOF
{"controller": "$C", "cap_label": "$CAP", "layout": "$LAYOUT", "note": "$NOTE",
 "started": "$TS", "stack_log": "$latest_log",
 "stack_log_header": "$(head -1 "$latest_log" 2>/dev/null | tr -d '"')",
 "mpc_max_speed_mps": "$mpc_cap", "pp_phase_multiplier": "$phase",
 "pp_sector_scaling": "$sector", "pp_default_scaling": "$default_scaling"}
EOF
T="/car_state/odom /car_state/odom_frenet /pf/pose/odom /vesc/odom /vesc/sensors/imu_corrected /imu/data \
/vesc/high_level/ackermann_cmd /vesc/ackermann_cmd /vesc/sensors/core /state_machine \
/simple_mux/active_host /simple_mux/estop_state /scan /lap_data /controller/latency \
/controller/mpc_status /controller/mpc_elapsed_ms /controller/mpc_cycle_ms /controller/mpc_loop_period_ms \
/controller/mpc_diagnostics /controller/mpc_predicted_path /tracking/obstacles /local_waypoints /global_waypoints \
/behavior_strategy /state_machine/debug /state_machine/trailing_target"
tmux new-session -d -s rec "bash -c 'source /opt/ros/jazzy/setup.bash; source $H/JSND/install/local_setup.bash; \
source $H/JSND_INT/install/local_setup.bash; \
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$H/JSND/src/racing_stack/cyclonedds.xml; \
ros2 bag record -s mcap -o $OUT $T; exec bash'"
echo "recording -> $OUT"
echo "meta      -> $OUT.meta.json"
cat "$OUT.meta.json"
echo "stop  : tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec"
