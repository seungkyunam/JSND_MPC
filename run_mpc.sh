#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 ISOLATED_BUILD_DIR MAP [mpc|pp] [--ros-distro auto|humble|jazzy]" >&2; exit 2
fi
build_root=$(realpath -e "$1")
map=$2
shift 2
controller=mpc
if [[ ${1:-} == mpc || ${1:-} == pp ]]; then controller=$1; shift; fi
distro=auto
if (( $# )); then
  if [[ $# != 2 || $1 != --ros-distro ]]; then echo 'Invalid arguments.' >&2; exit 2; fi
  distro=$2
fi
if [[ $distro != auto && $distro != humble && $distro != jazzy ]]; then echo 'Unsupported ROS distribution.' >&2; exit 2; fi
if [[ ! $map =~ ^[A-Za-z0-9_-]+$ || ( $controller != mpc && $controller != pp ) ]]; then
  echo 'Invalid map or controller.' >&2; exit 2
fi
if [[ ${SIM_ONLY_ISOLATED:-} != YES || ! ${ROS_DOMAIN_ID:-} =~ ^[0-9]{1,3}$ ]] || (( 10#${ROS_DOMAIN_ID:-0} < 1 || 10#${ROS_DOMAIN_ID:-0} > 232 )); then
  echo 'Simulation only: physically isolate the vehicle/ROS graph; set SIM_ONLY_ISOLATED=YES and a free nonzero ROS_DOMAIN_ID.' >&2; exit 1
fi
for variable in ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; do
  if [[ -n ${!variable:-} ]]; then echo "Refusing custom DDS discovery/configuration: $variable" >&2; exit 1; fi
done
if [[ ! -f "$build_root/build-complete.txt" || $(cat "$build_root/build-complete.txt") != sim-only || ! -r "$build_root/underlay-workspace.txt" ]]; then
  echo 'Not a completed simulation-only build.' >&2; exit 1
fi
workspace=$(cat "$build_root/underlay-workspace.txt")
stage=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "$stage/mpc_deployment/ros_distro.bash"
detected=$(mpc_detect_distro "$workspace" "$distro")
if [[ -f "$build_root/ros-distro.txt" && $(cat "$build_root/ros-distro.txt") != "$detected" ]]; then
  echo 'Build and underlay ROS distributions differ.' >&2; exit 1
fi
source "/opt/ros/$detected/setup.bash"
source "$workspace/install/local_setup.bash"
source "$build_root/venv/bin/activate"
source "$build_root/install/local_setup.bash"
if [[ ${ROS_DISTRO:-} != "$detected" ]]; then echo 'ROS distribution mismatch.' >&2; exit 1; fi
export PYTHONDONTWRITEBYTECODE=1
for variable in ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; do
  if [[ -n ${!variable:-} ]]; then echo "Refusing custom DDS discovery/configuration: $variable" >&2; exit 1; fi
done
if [[ $detected == jazzy ]]; then
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
else
  export ROS_LOCALHOST_ONLY=1
fi
share="$(ros2 pkg prefix stack_master)/share/stack_master"
for file in "$map.yaml" global_waypoints.json speed_scaling.yaml ot_sectors.yaml; do
  if [[ ! -f "$share/maps/$map/$file" ]]; then echo "Missing simulation map file: $share/maps/$map/$file" >&2; exit 1; fi
done
nodes=$(ros2 node list --no-daemon --spin-time 3 2>&1) || { echo "Cannot verify the isolated ROS domain: $nodes" >&2; exit 1; }
if [[ -n $nodes ]]; then echo "Refusing a nonempty simulation ROS domain: $nodes" >&2; exit 1; fi
echo 'Simulation only. ROS domain checks do not replace physical isolation.'
exec ros2 launch stack_master race.launch.xml "map:=$map" sim:=true "controller:=$controller" virtual:=false gp_predictor:=false
