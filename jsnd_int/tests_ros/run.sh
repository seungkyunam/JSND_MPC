#!/bin/bash
# Usage: bash tests_ros/run.sh <pp|mpc> <safety|ftg|estop> [extra launch args, e.g. safety_mode:=false mpc_rate_hz:=40]
# Isolated domain 219, no VESC/sensors.
cd ~/JSND_INT
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=219 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
C=$1; S=$2; shift 2
L=${LOG_DIR:-/tmp}/ros_${C}_${S}.log
ros2 launch tests_ros/ctrl_mux_only.launch.xml controller:=$C "$@" > $L 2>&1 &
sleep 7
python3 tests_ros/fake_world.py $S 2>&1 | grep -v "optional interface"
pkill -INT -f "simple_mux_node.py|lib/controller/controller_manager|ros2 launch tests_ros"; sleep 2
grep -v "optional interface" $L | grep -E "MPC selected|STOP requested|tracking local|FTGONLY|odom stale|no local|EStop|safety|ERROR|Traceback|died" \
  | sed -E 's/IntegratedConfig\(.*/IntegratedConfig(...)/; s/^\[[^]]*\] \[[A-Z]+\] \[[0-9.]+\] //' | uniq -c | head -40
