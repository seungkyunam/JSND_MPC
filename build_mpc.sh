#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 ABSOLUTE_UNDERLAY_WORKSPACE [--install-deps] [--ros-distro auto|humble|jazzy]" >&2; exit 2
fi
stage=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
workspace=$(realpath -e "$1")
shift
install_deps=false
distro=auto
while (( $# )); do
  case $1 in
    --install-deps) install_deps=true; shift ;;
    --ros-distro) [[ $# -ge 2 ]] || exit 2; distro=$2; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ $distro != auto && $distro != humble && $distro != jazzy ]]; then
  echo "Unsupported ROS distribution: $distro" >&2; exit 2
fi
if [[ "$stage/" == "$workspace/"* || "$workspace/" == "$stage/"* ]]; then
  echo 'Refusing: staging directory and underlay overlap.' >&2; exit 1
fi
if [[ ! -f "$stage/controller/package.xml" || ! -f "$stage/stack_master/launch/validate_controller.launch.py" ]]; then
  echo 'Incomplete simulation source stage.' >&2; exit 1
fi
source "$stage/mpc_deployment/ros_distro.bash"
distro=$(mpc_detect_distro "$workspace" "$distro")
build_root=$(mktemp -d "$stage/mpc-build.XXXXXX")
echo "Isolated build directory: $build_root"
source "/opt/ros/$distro/setup.bash"
source "$workspace/install/local_setup.bash"
if [[ ${ROS_DISTRO:-} != "$distro" ]]; then echo 'ROS distribution mismatch.' >&2; exit 1; fi
python3 -m venv --system-site-packages "$build_root/venv"
source "$build_root/venv/bin/activate"
export PYTHONDONTWRITEBYTECODE=1
if [[ $install_deps == true ]]; then
  python -m pip install -r "$stage/mpc_deployment/requirements.txt"
fi
python - <<'PY'
import numpy, scipy, osqp
from importlib.metadata import version
from packaging.specifiers import SpecifierSet

for name, spec in [('numpy', '>=1.23,<2'), ('scipy', '>=1.9'), ('osqp', '>=1.0,<2')]:
    installed = version(name)
    print(f'{name} {installed} (required {spec})')
    if installed not in SpecifierSet(spec):
        raise SystemExit(f'Incompatible {name} version: {installed}')
PY
python -c 'import rclpy, ackermann_msgs.msg, f110_msgs.msg, frenet_conversion; print("ROS Python imports OK")'
python -m colcon build --help >/dev/null
printf '%s\n' "$workspace" > "$build_root/underlay-workspace.txt"
printf '%s\n' "$distro" > "$build_root/ros-distro.txt"
python -m colcon --log-base "$build_root/log" build \
  --base-paths "$stage/controller" "$stage/stack_master" \
  --build-base "$build_root/build" --install-base "$build_root/install" \
  --packages-select controller stack_master --allow-overriding controller stack_master \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  2>&1 | tee "$build_root/build-console.log"
test -r "$build_root/install/local_setup.bash"
launcher="$build_root/install/controller/lib/controller/controller_manager"
if [[ ! -f $launcher || $(head -n 1 "$launcher") != "#!$VIRTUAL_ENV/bin/"* ]]; then
  echo 'Controller executable does not use the isolated Python environment; refusing to mark build complete.' >&2; exit 1
fi
printf '%s\n' 'sim-only' > "$build_root/build-complete.txt"
echo "Build completed: $build_root"
echo "Simulation only: SIM_ONLY_ISOLATED=YES ROS_DOMAIN_ID=213 bash $stage/run_mpc.sh $build_root test0 mpc"
