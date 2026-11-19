#!/usr/bin/env bash
# Clone/update JSND_MPC, build the isolated Jazzy MPC vehicle overlay, and
# optionally start only the fail-closed Shadow launch. Never starts actuators.
set -euo pipefail

REPO_URL=${MPC_REPO_URL:-https://github.com/seungkyunam/JSND_MPC.git}
REPO_DIR=${MPC_REPO_DIR:-"$HOME/JSND_MPC"}
UNDERLAY=${MPC_JAZZY_UNDERLAY:-"$HOME/JSND"}
PROFILE=${MPC_PROFILE:-}
DOMAIN=${ROS_DOMAIN_ID:-}
RUN_SHADOW=0
SKIP_ROSDEP=0

usage() {
  cat >&2 <<'EOF'
Usage: install_jazzy_shadow.sh [options]

Options:
  --repo-dir DIR       clone/update destination (default: $HOME/JSND_MPC)
  --underlay DIR       existing Jazzy workspace (default: $HOME/JSND)
  --profile FILE       calibrated vehicle.json (required for --run-shadow)
  --domain ID          isolated ROS domain 0..232 (required for --run-shadow)
  --run-shadow         start mpc_vehicle Shadow after a successful build
  --skip-rosdep        skip rosdep install (only for an already provisioned host)
EOF
}

while (($#)); do
  case $1 in
    --repo-dir) [[ $# -ge 2 ]] || { usage; exit 2; }; REPO_DIR=$2; shift 2 ;;
    --underlay) [[ $# -ge 2 ]] || { usage; exit 2; }; UNDERLAY=$2; shift 2 ;;
    --profile) [[ $# -ge 2 ]] || { usage; exit 2; }; PROFILE=$2; shift 2 ;;
    --domain) [[ $# -ge 2 ]] || { usage; exit 2; }; DOMAIN=$2; shift 2 ;;
    --run-shadow) RUN_SHADOW=1; shift ;;
    --skip-rosdep) SKIP_ROSDEP=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

REPO_DIR=$(realpath -m "$REPO_DIR")
UNDERLAY=$(realpath -m "$UNDERLAY")

if [[ ! -d "$REPO_DIR/.git" ]]; then
  [[ ! -e "$REPO_DIR" ]] || { echo "Refusing non-git existing path: $REPO_DIR" >&2; exit 1; }
  git clone --branch main --single-branch "$REPO_URL" "$REPO_DIR"
else
  git -C "$REPO_DIR" diff --quiet && git -C "$REPO_DIR" diff --cached --quiet \
    || { echo "Refusing to update a dirty repository: $REPO_DIR" >&2; exit 1; }
  git -C "$REPO_DIR" pull --ff-only origin main
fi

STAGE=$(cd "$REPO_DIR" && pwd -P)
[[ -r /opt/ros/jazzy/setup.bash ]] || {
  echo 'ROS 2 Jazzy is not installed: /opt/ros/jazzy/setup.bash is missing.' >&2; exit 1;
}
[[ -r "$UNDERLAY/install/setup.bash" ]] || {
  echo "Jazzy underlay setup is missing: $UNDERLAY/install/setup.bash" >&2; exit 1;
}
[[ -r "$UNDERLAY/install/local_setup.bash" ]] || {
  echo "Jazzy underlay local setup is missing: $UNDERLAY/install/local_setup.bash" >&2; exit 1;
}

# A fresh shell is required. Do not allow a Humble or other ROS prefix to leak
# into the isolated vehicle build.
if [[ -n ${ROS_DISTRO:-} && ${ROS_DISTRO} != jazzy ]]; then
  echo "Current ROS_DISTRO=${ROS_DISTRO}; start a fresh shell for Jazzy." >&2; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$UNDERLAY/install/local_setup.bash"
[[ ${ROS_DISTRO:-} == jazzy ]] || { echo 'Jazzy underlay did not set ROS_DISTRO=jazzy.' >&2; exit 1; }
source "$STAGE/mpc_deployment/ros_distro.bash"
mpc_detect_distro "$UNDERLAY" jazzy >/dev/null
python3 -c 'import rclpy, sys; print("rclpy", rclpy.__file__)' >/dev/null || {
  echo 'Jazzy Python environment cannot import rclpy.' >&2; exit 1;
}
for var in ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER CYCLONEDDS_URI \
           FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS; do
  [[ -z ${!var:-} ]] || { echo "Refusing inherited ROS discovery variable: $var" >&2; exit 1; }
done

if ((SKIP_ROSDEP == 0)); then
  command -v rosdep >/dev/null || { echo 'rosdep is required.' >&2; exit 1; }
  rosdep install --from-paths "$STAGE/controller" "$STAGE/mpc_vehicle" \
    --ignore-src --rosdistro jazzy -r -y
fi

[[ -f "$STAGE/mpc_deployment/requirements.txt" ]] || exit 1
build_log=$(mktemp "${TMPDIR:-/tmp}/jsnd-mpc-jazzy-build.XXXXXX")
trap 'rm -f "$build_log"' EXIT
bash "$STAGE/build_mpc_vehicle.sh" "$UNDERLAY" | tee "$build_log"
BUILD=$(sed -n 's/^Vehicle build completed: //p' "$build_log" | tail -n 1)
[[ -n "$BUILD" && -f "$BUILD/vehicle-build-complete.txt" ]] || {
  echo 'Vehicle build did not produce a commissioning marker.' >&2; exit 1;
}

echo "Jazzy MPC vehicle build ready: $BUILD"
if ((RUN_SHADOW)); then
  [[ -n "$PROFILE" && -f "$PROFILE" ]] || {
    echo '--profile FILE is required for --run-shadow; no calibration defaults are invented.' >&2; exit 2;
  }
  [[ "$DOMAIN" =~ ^[0-9]{1,3}$ ]] && ((10#$DOMAIN <= 232)) || {
    echo '--domain ID in 0..232 is required for Shadow.' >&2; exit 2;
  }
  export ROS_DOMAIN_ID=$DOMAIN
  exec bash "$STAGE/run_mpc_vehicle.sh" "$BUILD" "$(realpath -e "$PROFILE")" shadow
fi

cat <<EOF
Shadow is not started automatically. To run it in this Jazzy shell:
  export ROS_DOMAIN_ID=<isolated-domain-0..232>
  bash $STAGE/run_mpc_vehicle.sh $BUILD /absolute/path/to/vehicle.json shadow
EOF
