#!/usr/bin/env bash
# Add vehicle package to a new isolated build; original simulation builder stays unchanged.
set -eo pipefail
if [[ $# != 1 ]]; then echo "Usage: $0 ABSOLUTE_JAZZY_UNDERLAY" >&2; exit 2; fi
stage=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
underlay=$(realpath -e "$1")
source "$stage/mpc_deployment/ros_distro.bash"
mpc_detect_distro "$underlay" jazzy >/dev/null
export PYTHONDONTWRITEBYTECODE=1
log=$(mktemp /tmp/mpc-vehicle-build.XXXXXX)
bash "$stage/build_mpc.sh" "$underlay" --install-deps --ros-distro jazzy | tee "$log"
build_root=$(sed -n 's/^Build completed: //p' "$log" | tail -n 1)
test -n "$build_root"
source /opt/ros/jazzy/setup.bash
source "$underlay/install/local_setup.bash"
source "$build_root/venv/bin/activate"
source "$build_root/install/local_setup.bash"
python -m colcon --log-base "$build_root/vehicle-log" build \
  --base-paths "$stage/mpc_vehicle" --packages-select mpc_vehicle \
  --build-base "$build_root/vehicle-build" --install-base "$build_root/install" \
  --executor sequential
launcher="$build_root/install/mpc_vehicle/lib/mpc_vehicle/mpc_vehicle_gate"
[[ $(head -n 1 "$launcher") == "#!$VIRTUAL_ENV/bin/"* ]] || { echo 'Wrong gate Python interpreter' >&2; exit 1; }
printf '%s\n' jazzy-vehicle-commissioning > "$build_root/vehicle-build-complete.txt"
echo "Vehicle build completed: $build_root"
echo 'Default shadow mode has no hardware output. Hardware and approval checks are still required.'
