#!/usr/bin/env bash
# usage: bash ~/JSND_INT/compare_pp_mpc/cmp_analyze.sh --bags <bag>... [--lapfiles] [--set k=v ...] --out <dir>
# Read-only: sources ROS + JSND (f110_msgs) and runs cmp_analyze.py (CSV + summary.md + PNG).
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
source "$HOME/JSND_INT/install/local_setup.bash"
exec python3 "$(dirname "${BASH_SOURCE[0]}")/cmp_analyze.py" "$@"
