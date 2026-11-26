#!/usr/bin/env bash
# Jetson: one tmux session "mpc" for wheels-up bench + laptop pitwall.
#   window 1 joy     : joy_node (loopback, domain 173)
#   window 2 bench   : run_mpc_vehicle.sh ... bench MAP  -> waits for YOUR WHEELS_LIFTED input
#   window 3 relay   : mpc_pitwall_relay (173 <-> 174)
#   window 4 status  : read-only gate/controller status
# Nothing here arms the gate or commands motors. Re-running just re-attaches.
# From the laptop:  ssh -t jsnd@<JETSON_IP> JSND_MPC/pitwall/mpc_bench_session.sh [MAP]
# Stop everything:  JSND_MPC/pitwall/mpc_bench_session.sh stop
set -eo pipefail
MAP=${1:-11_18_6}
BUILD=$HOME/JSND_MPC/mpc-build.WFANmz
PROFILE=$HOME/JSND_MPC/mpc-build.vehicle-assets/vehicle.json
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
S=mpc

if [[ $MAP == stop ]]; then
  tmux has-session -t $S 2>/dev/null || { echo "no session '$S'"; exit 0; }
  echo "Ctrl-C to bench (gate publishes zero command on exit), then relay/joy/status..."
  tmux send-keys -t $S:bench C-c; sleep 4
  for w in relay status joy; do tmux send-keys -t $S:$w C-c 2>/dev/null || true; done; sleep 2
  tmux kill-session -t $S
  pgrep -af 'vesc_driver|urg_node|particle_filter|mpc_vehicle|joy_node|mpc_pitwall_relay' || echo "all stopped"
  exit 0
fi

if ! tmux has-session -t $S 2>/dev/null; then
  ENV173="source /opt/ros/jazzy/setup.bash; source $HOME/JSND/install/local_setup.bash; \
source $BUILD/venv/bin/activate; source $BUILD/install/local_setup.bash; \
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; unset ROS_LOCALHOST_ONLY; \
export CYCLONEDDS_URI=file://$BUILD/install/mpc_vehicle/share/mpc_vehicle/config/cyclonedds_vehicle_localhost.xml"
  tmux new-session -d -s $S -n joy
  tmux send-keys -t $S:joy "$ENV173; ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=50.0" C-m
  tmux new-window -d -t $S -n relay
  tmux send-keys -t $S:relay "$HERE/mpc_pitwall_relay.sh" C-m
  tmux new-window -d -t $S -n status
  tmux send-keys -t $S:status "$ENV173; sleep 15; ros2 topic echo --no-daemon /mpc_vehicle/gate_status --field data" C-m
  tmux new-window -t $S -n bench
  tmux send-keys -t $S:bench "sleep 3; export ROS_DOMAIN_ID=173; bash $HOME/JSND_MPC/run_mpc_vehicle.sh $BUILD $PROFILE bench $MAP" C-m
  tmux select-window -t $S:bench
fi
cat <<'EOF'
[tmux 'mpc']  Ctrl-b 0 joy | 1 relay | 2 status | 3 bench | Ctrl-b w list   (Ctrl-b d = detach, keeps running)
bench window: type WHEELS_LIFTED only after the wheels are really off the ground and the physical estop is at hand.
EOF
sleep 1
exec tmux attach -t $S
