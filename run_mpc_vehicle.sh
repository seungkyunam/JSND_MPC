#!/usr/bin/env bash
set -eo pipefail
if [[ $# -lt 2 || $# -gt 4 ]]; then
  echo "Usage: $0 BUILD_DIR PROFILE_JSON [shadow|bench|drive] [MAP]" >&2
  echo "  shadow: no MAP; follows /global_waypoints of the map the running stack selected." >&2
  echo "  bench|drive: MAP = map name under the underlay's stack_master maps, or a map directory." >&2
  exit 2
fi
build_root=$(realpath -e "$1")
profile=$(realpath -e "$2")
mode=${3:-shadow}
case "$mode" in shadow|bench|drive) ;; *) echo 'Invalid mode' >&2; exit 2;; esac
[[ $(cat "$build_root/vehicle-build-complete.txt") == jazzy-vehicle-commissioning ]] || exit 1
[[ $(cat "$build_root/ros-distro.txt") == jazzy ]] || exit 1
stage=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
underlay=$(cat "$build_root/underlay-workspace.txt")
map_arg=${4:-}
map_dir=''
if [[ $mode == shadow ]]; then
  [[ -z $map_arg ]] || { echo 'shadow takes no MAP: it follows the running stack /global_waypoints' >&2; exit 2; }
else
  [[ -n $map_arg ]] || { echo "$mode requires MAP (no map is stored in the vehicle profile)" >&2; exit 2; }
  if [[ $map_arg == */* ]]; then
    map_dir=$(realpath -e "$map_arg")
  else
    map_dir=$(realpath -e "$underlay/install/stack_master/share/stack_master/maps/$map_arg")
  fi
fi
source "$stage/mpc_deployment/ros_distro.bash"
mpc_detect_distro "$underlay" jazzy >/dev/null
source /opt/ros/jazzy/setup.bash
source "$underlay/install/local_setup.bash"
source "$build_root/venv/bin/activate"
source "$build_root/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
[[ ${ROS_DOMAIN_ID:-} =~ ^[0-9]{1,3}$ ]] && (( 10#$ROS_DOMAIN_ID <= 232 )) || {
  echo 'Explicit ROS_DOMAIN_ID (0..232) required; match local safety/joy nodes' >&2; exit 1;
}
for variable in ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS; do
  [[ -z ${!variable:-} ]] || { echo "Custom DDS not supported for commissioning: $variable" >&2; exit 1; }
done
# The vehicle stack runs CycloneDDS with SPDP multicast on loopback; the
# built-in LOCALHOST profile cannot discover it. Use the packaged loopback-only
# profile instead of whatever CYCLONEDDS_URI the shell inherited.
if [[ -n ${RMW_IMPLEMENTATION:-} && $RMW_IMPLEMENTATION != rmw_cyclonedds_cpp ]]; then
  echo "Commissioning requires rmw_cyclonedds_cpp (got $RMW_IMPLEMENTATION)" >&2; exit 1
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
dds_profile="$build_root/install/mpc_vehicle/share/mpc_vehicle/config/cyclonedds_vehicle_localhost.xml"
[[ -r $dds_profile ]] || { echo "Missing packaged DDS profile: $dds_profile" >&2; exit 1; }
if [[ -n ${CYCLONEDDS_URI:-} && $CYCLONEDDS_URI != "file://$dds_profile" ]]; then
  echo "Replacing inherited CYCLONEDDS_URI ($CYCLONEDDS_URI) with loopback-only $dds_profile" >&2
fi
export CYCLONEDDS_URI="file://$dds_profile"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
confirm=''
if [[ $mode == bench ]]; then
  read -r -p 'Physical estop tested, vehicle restrained with ALL wheels lifted? Type WHEELS_LIFTED: ' confirm
  [[ $confirm == WHEELS_LIFTED ]] || exit 1
elif [[ $mode == drive ]]; then
  read -r -p 'Wheels-up approval complete, controlled low-speed test area clear? Type CONTROLLED_LOW_SPEED_TEST: ' confirm
  [[ $confirm == CONTROLLED_LOW_SPEED_TEST ]] || exit 1
fi
# ros2 launch rejects empty 'name:=' arguments; omit unset ones (launch defaults are '').
launch_args=("profile:=$profile" "mode:=$mode")
[[ -z $confirm ]] || launch_args+=("confirm:=$confirm")
[[ -z $map_dir ]] || launch_args+=("map_dir:=$map_dir")
exec ros2 launch mpc_vehicle vehicle_mpc.launch.py "${launch_args[@]}"
