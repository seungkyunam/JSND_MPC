#!/usr/bin/env bash
# Jetson: start the MPC pitwall relay (domain 173 loopback <-> 174 WiFi).
# Run in its own terminal (e.g. over SSH from the laptop, no X11 needed),
# AFTER or BEFORE run_mpc_vehicle.sh -- order does not matter.
# It never starts drivers and never publishes drive/gate/estop topics.
set -eo pipefail
here=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
iface=wlP1p1s0
ip -4 -br addr show "$iface" | grep -q UP || { echo "WiFi $iface is not up" >&2; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"   # f110_msgs (WpntArray)
for variable in ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS ROS_LOCALHOST_ONLY; do
  unset "$variable"
done
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Per-domain interfaces come from the XML (173 = 127.0.0.1 only, 174 = WiFi only).
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI="file://$here/cyclonedds_mpc_pitwall_relay.xml"
export PYTHONDONTWRITEBYTECODE=1
echo "Jetson WiFi: $(ip -4 -br addr show "$iface" | awk '{print $3}')  (vehicle 173 loopback <-> pitwall 174)"
exec python3 "$here/mpc_pitwall_relay.py"
