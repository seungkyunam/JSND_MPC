#!/usr/bin/env bash
# Step 18: launch JSND race stack with integrated MPC at a given speed profile, logging for stall comparison.
# usage: bash ~/JSND_INT/run/race_mpc.sh <2p25|3p0|4p0|5p0> [map=11_18_6]
# Runs in tmux session "race" (foreground log tee). Stop: Ctrl-C in tmux, or `tmux kill-session -t race`.
set -eo pipefail
PROFILE=${1:?profile: 2p25|3p0|4p0|5p0}; MAP=${2:-11_18_6}
[ -f "$HOME/JSND_INT/install/stack_master/share/stack_master/config/mpc_profile_${PROFILE}.yaml" ] || { echo "unknown profile $PROFILE"; exit 1; }
# single stack only: refuse if another race/vesc stack is alive (yesterday's double-VESC incident)
if pgrep -f "vesc_driver_node|race.launch.xml" >/dev/null; then echo "ABORT: another stack running:"; pgrep -af "vesc_driver_node|race.launch.xml"; exit 1; fi
tmux has-session -t race 2>/dev/null && { echo "ABORT: tmux session 'race' exists"; exit 1; }
TS=$(date +%Y%m%d_%H%M%S); LOG=$HOME/bench_logs/race_mpc${PROFILE}_${TS}.log
CMD="source /opt/ros/jazzy/setup.bash; source \$HOME/JSND/install/local_setup.bash; source \$HOME/JSND_INT/install/local_setup.bash; \
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://\$HOME/JSND/src/racing_stack/cyclonedds.xml; \
echo \"commit \$(git -C \$HOME/JSND_INT/src/racing_stack rev-parse --short HEAD) profile=$PROFILE map=$MAP\" | tee $LOG; \
ros2 launch stack_master race.launch.xml map:=$MAP controller:=mpc use_estop:=true mpc_rate_hz:=50 mpc_profile:=$PROFILE 2>&1 | tee -a $LOG"
tmux new-session -d -s race "bash -c '$CMD; exec bash'"
echo "started tmux 'race'  profile=$PROFILE map=$MAP"
echo "log : $LOG"
echo "view: tmux attach -t race      summary: python3 ~/JSND_INT/run/stall_summary.py $LOG"
