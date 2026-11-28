#!/usr/bin/env bash
# Final PP/MPC vehicle launcher. Invoked by the operator ON Jetson only.
set -eo pipefail
mode=${1:?usage: step45_run.sh pp|mpc [map=11_18_6] [profile=lowspeed|1p5|2p25|3p0|4p0|5p0]}
map_name=${2:-11_18_6}
profile=${3:-lowspeed}
case "$mode" in pp|mpc) ;; *) echo 'mode must be pp or mpc'; exit 2;; esac
case "$profile" in lowspeed|1p5|2p25|3p0|4p0|5p0) ;; *) echo 'profile must be a staged speed-only profile'; exit 2;; esac
[[ "$map_name" =~ ^[a-zA-Z0-9_-]+$ && "$profile" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
if pgrep -f '[v]esc_driver_node|[r]ace.launch.xml' >/dev/null; then
  echo 'Another vehicle stack is running. Stop its launch before switching modes.'; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
vehicle_overlay=${JSND_INT_OVERLAY:-$HOME/JSND_INT}
source "$vehicle_overlay/install/local_setup.bash"
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/JSND/src/racing_stack/cyclonedds.xml"
installed_stack=$(ros2 pkg prefix stack_master)/share/stack_master
[[ -f "$installed_stack/maps/$map_name/$map_name.yaml" ]] || { echo 'Installed map missing'; exit 1; }
[[ -f "$installed_stack/config/mpc_profile_$profile.yaml" ]] || { echo 'Installed profile missing'; exit 1; }
mkdir -p "$HOME/bench_logs"
log_path="$HOME/bench_logs/step45_${mode}_${profile}_$(date +%Y%m%d_%H%M%S).log"
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
echo "checkout=$(git -C "$repo_root" rev-parse HEAD) mode=$mode map=$map_name profile=$profile" | tee "$log_path"
args=(map:="$map_name" sim:=false localization:=pf)
if [[ "$mode" == pp ]]; then
  # Preserve PP control and its existing safety-mode default.
  args+=(controller:=pp)
else
  # Final c16bced vehicle release: fixed integrated MPC, 50 Hz, N=10, 1.0 s.
  # race.launch validates these values and refuses any incompatible combination.
  args+=(controller:=mpc mpc_speed_source:=integrated mpc_rate_hz:=50
         mpc_profile:="$profile" mpc_monitor:=false
         mpc_heading_robust:=false mpc_history_prediction:=true safety_mode:=true)
fi
ros2 launch stack_master race.launch.xml "${args[@]}" 2>&1 | tee -a "$log_path"
