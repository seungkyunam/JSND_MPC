#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="/home/sk/rober/racing_stack/install/f110_msgs/local/lib/python3.10/dist-packages:${ROOT}/planner/spliner:${ROOT}/race_utils/f110_utils/libs/frenet_conversion:${ROOT}/race_utils/f110_utils/libs/grid_filter:${ROOT}/race_utils/f110_utils/libs/vel_planner:${PYTHONPATH:-}"
exec /home/sk/JSND_MPC/.venv/bin/python "$@"
