#!/usr/bin/env bash
# Compatibility launcher for the final c16bced vehicle MPC.
# usage: bash run/race_mpc.sh <lowspeed|1p5|2p25|3p0|4p0|5p0|6p0|7p0|8p0|9p0|10p0> [map=11_18_6]
# Runs in tmux session "race" (foreground log tee). Stop: Ctrl-C in tmux, or `tmux kill-session -t race`.
set -eo pipefail
PROFILE=${1:?profile: lowspeed|1p5|2p25|3p0|4p0|5p0|6p0|7p0|8p0|9p0|10p0}; MAP=${2:-11_18_6}
[[ $# -le 2 ]] || { echo "ABORT: speed source is fixed to integrated"; exit 2; }
case $PROFILE in lowspeed|1p5|2p25|3p0|4p0|5p0|6p0|7p0|8p0|9p0|10p0) ;; *) echo "ABORT: staged speed-only profile required"; exit 2;; esac
[ "$(id -u)" -eq 0 ] && { echo "ABORT: run as jsnd, not root/sudo"; exit 1; }
[ "$(hostname)" = "jsnd" ] || { echo "ABORT: run on Jetson (ssh jsnd@172.20.10.4)"; exit 1; }
vehicle_overlay=${JSND_INT_OVERLAY:-$HOME/JSND_INT}
CFG=$vehicle_overlay/install/stack_master/share/stack_master/config
[ -f "$CFG/mpc_profile_${PROFILE}.yaml" ] || { echo "unknown profile $PROFILE (in $CFG):"; ls "$CFG" | sed -n 's/^mpc_profile_\(.*\)\.yaml$/  \1/p'; exit 1; }
[ -f "$vehicle_overlay/install/stack_master/share/stack_master/maps/$MAP/$MAP.yaml" ] || { echo "unknown map $MAP"; exit 1; }
# single stack only: refuse if another race/vesc stack is alive (yesterday's double-VESC incident)
if pgrep -f "vesc_driver_node|race.launch.xml" >/dev/null; then echo "ABORT: another stack running:"; pgrep -af "vesc_driver_node|race.launch.xml"; exit 1; fi
tmux has-session -t race 2>/dev/null && { echo "ABORT: tmux session 'race' exists"; exit 1; }
TS=$(date +%Y%m%d_%H%M%S); LOG=$HOME/bench_logs/race_mpc${PROFILE}_integrated_${TS}.log
CMD="source /opt/ros/jazzy/setup.bash; source \$HOME/JSND/install/local_setup.bash; source $vehicle_overlay/install/local_setup.bash; \
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://\$HOME/JSND/src/racing_stack/cyclonedds.xml; \
echo \"commit \$(git -C $vehicle_overlay/src/racing_stack rev-parse --short HEAD) profile=$PROFILE map=$MAP source=integrated final=c16bced\" | tee $LOG; \
ros2 launch stack_master race.launch.xml map:=$MAP controller:=mpc safety_mode:=true mpc_rate_hz:=50 mpc_profile:=$PROFILE mpc_speed_source:=integrated mpc_heading_robust:=false mpc_history_prediction:=true 2>&1 | tee -a $LOG"
tmux new-session -d -s race "bash -c '$CMD; exec bash'"
echo "started tmux 'race'  profile=$PROFILE map=$MAP source=integrated final=c16bced"
echo "log : $LOG"
echo "view: tmux attach -t race      summary: python3 ~/JSND_INT/run/stall_summary.py $LOG"
