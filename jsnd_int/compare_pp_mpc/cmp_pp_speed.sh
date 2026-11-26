#!/usr/bin/env bash
# Set PP's speed scale at runtime WITHOUT code/file changes (speed_sector_tuner live param).
# usage: bash ~/JSND_INT/compare_pp_mpc/cmp_pp_speed.sh <target_cap_mps> [raceline_vmax=auto]
# phase_multiplier = target_cap / (raceline vx max * max sector scaling). It scales the whole
# PP speed profile (shape preserved); it is NOT a clip like the MPC profile cap. The analyzer
# reports the measured max speed of each run so the cap match can be checked afterwards.
# Only affects PP (integrated MPC ignores raceline speeds). Resets on stack restart.
set -eo pipefail
CAP=${1:?target cap m/s}; VMAX=${2:-auto}
H=$HOME
source /opt/ros/jazzy/setup.bash; source $H/JSND/install/local_setup.bash; source $H/JSND_INT/install/local_setup.bash
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$H/JSND/src/racing_stack/cyclonedds.xml
MAPDIR=$(ros2 pkg prefix stack_master)/share/stack_master/maps/${MAP:-11_18_6}
if [[ "$VMAX" == auto ]]; then
  VMAX=$(python3 - "$MAPDIR" <<'EOF'
import json, sys, yaml, os
d = sys.argv[1]
w = json.load(open(os.path.join(d, 'global_waypoints.json')))
# iqp raceline = /global_waypoints on the car (memory: not sp)
v = max(p['vx_mps'] for p in w['global_traj_wpnts_iqp']['wpnts'])
s = yaml.safe_load(open(os.path.join(d, 'speed_scaling.yaml')))['speed_sector_tuner']['ros__parameters']
sc = max(s[k]['scaling'] for k in s if k.startswith('Sector'))
print(v * sc)
EOF
)
fi
PHASE=$(python3 -c "print(round(min(2.0, $CAP / $VMAX), 4))")
echo "raceline vmax*sector_max=$VMAX m/s -> phase_multiplier=$PHASE (target $CAP m/s)"
read -r -p "apply to /speed_sector_tuner? [y/N] " ok
[[ "$ok" == y ]] || exit 0
ros2 param set /speed_sector_tuner phase_multiplier "$PHASE"
ros2 param get /speed_sector_tuner phase_multiplier
