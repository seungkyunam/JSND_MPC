#!/usr/bin/env bash
# Local Humble simulation convenience launcher; never starts vehicle drivers.
set -eo pipefail
stage=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
cd "$stage"
if [[ ${SIM_ONLY_ISOLATED:-} != YES ]]; then
  echo 'Use an isolated simulation session: SIM_ONLY_ISOLATED=YES bash run_test0_sim.sh' >&2
  exit 1
fi
source "$stage/controller/test/sim_environment.bash"
# Interactive terminals use the desktop; headless test sessions stay offscreen.
if [[ -n ${DISPLAY:-} ]]; then unset QT_QPA_PLATFORM; fi
exec bash "$stage/run_mpc.sh" "$stage/mpc-build.F39akS" test0 mpc --ros-distro humble
