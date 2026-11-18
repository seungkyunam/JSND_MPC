#!/usr/bin/env bash
set -eo pipefail

usage() {
  echo "Usage: $0 VEHICLE_HOST USERNAME ABSOLUTE_WORKSPACE [--apply] [--ros-distro auto|humble|jazzy]" >&2
  echo "Default: inspect only; --apply creates a new ~/mpc_ws simulation-only stage." >&2
}

if [[ $# -lt 3 ]]; then usage; exit 2; fi
host=$1
user=$2
workspace=$3
shift 3
mode=--preview
distro=auto
while (( $# )); do
  case $1 in
    --apply|--preview) mode=$1; shift ;;
    --ros-distro) [[ $# -ge 2 ]] || { usage; exit 2; }; distro=$2; shift 2 ;;
    *) usage; exit 2 ;;
  esac
done
if [[ ! $host =~ ^[A-Za-z0-9][A-Za-z0-9.:-]*$ || ! $user =~ ^[A-Za-z_][A-Za-z0-9_-]*$ || ! $workspace =~ ^/[A-Za-z0-9_./-]+$ || /$workspace/ == *'/../'* || ( $distro != auto && $distro != humble && $distro != jazzy ) ]]; then
  usage; exit 2
fi
root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
target="$user@$host"
temp_dir=$(mktemp -d "${TMPDIR:-/tmp}/jsnd-mpc-inspect.XXXXXX")
python3 "$root/mpc_deployment/inspect_sources.py" scan "$root" > "$temp_dir/local.json"
ssh "$target" "python3 - scan $workspace/src" < "$root/mpc_deployment/inspect_sources.py" > "$temp_dir/remote.json"
python3 "$root/mpc_deployment/inspect_sources.py" compare "$temp_dir/local.json" "$temp_dir/remote.json"
detected=$(ssh "$target" "bash -s -- $workspace $distro" < <(cat "$root/mpc_deployment/ros_distro.bash"; printf '\nmpc_detect_distro "$1" "$2"\n'))
echo "Target underlay ROS: $detected"
echo "Inspection records: $temp_dir"
echo "Existing workspace is read-only to this script; this is NOT authorization for real vehicle MPC."
if [[ $mode == --preview ]]; then
  echo 'Preview only. Review every difference; rerun with --apply to create an isolated simulation stage.'
  exit 0
fi
read -r -p 'Review the report above. Type STAGE to create the separate simulation stage: ' confirmation
if [[ $confirmation != STAGE ]]; then echo 'Deployment cancelled.' >&2; exit 1; fi
remote_dir=$(ssh "$target" "bash -s -- $workspace" <<'REMOTE'
set -eo pipefail
workspace=$(realpath -e "$1")
home=$(realpath -e "$HOME")
stage="$home/mpc_ws"
if [[ "$stage/" == "$workspace/"* || "$workspace/" == "$stage/"* ]]; then
  echo 'Refusing: stage and existing workspace overlap.' >&2
  exit 1
fi
if [[ -e "$stage" || -L "$stage" ]]; then
  echo "Refusing to overwrite existing $stage." >&2; exit 1
fi
mkdir "$stage"
printf '%s\n' "$stage"
REMOTE
)
if [[ ! $remote_dir =~ ^/[A-Za-z0-9_./-]+$ ]]; then
  echo "Unsafe or unexpected remote staging path: $remote_dir" >&2; exit 1
fi
echo "Creating simulation-only stage: $remote_dir"
tar --exclude='.git' --exclude='build' --exclude='install' --exclude='log' --exclude='__pycache__' --exclude='.pytest_cache' --exclude='*.egg-info' --exclude='*.pyc' --exclude='*.pyo' \
  -C "$root" -cf - controller stack_master mpc_deployment build_mpc.sh run_mpc.sh |
  ssh "$target" "tar -xf - -C $remote_dir"
echo "Staged at $target:$remote_dir"
echo "On vehicle: cd $remote_dir && bash build_mpc.sh $workspace --ros-distro $detected"
echo 'Keep this overlay out of all real-driving shells; its MPC launch remains sim-only.'
