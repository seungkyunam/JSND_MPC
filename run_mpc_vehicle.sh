#!/usr/bin/env bash
set -eo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 BUILD_DIR PROFILE_JSON [shadow|bench|drive]" >&2; exit 2
fi
build_root=$(realpath -e "$1")
profile=$(realpath -e "$2")
mode=${3:-shadow}
case "$mode" in shadow|bench|drive) ;; *) echo 'Invalid mode' >&2; exit 2;; esac
[[ $(cat "$build_root/vehicle-build-complete.txt") == jazzy-vehicle-commissioning ]] || exit 1
[[ $(cat "$build_root/ros-distro.txt") == jazzy ]] || exit 1
stage=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
underlay=$(cat "$build_root/underlay-workspace.txt")
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
for variable in ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS; do
  [[ -z ${!variable:-} ]] || { echo "Custom DDS not supported for commissioning: $variable" >&2; exit 1; }
done
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
exec ros2 launch mpc_vehicle vehicle_mpc.launch.py "profile:=$profile" "mode:=$mode" "confirm:=$confirm"
