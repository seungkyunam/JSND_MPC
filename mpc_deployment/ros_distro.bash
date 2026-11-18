#!/usr/bin/env bash

mpc_detect_distro() {
  local workspace=$1 requested=$2 detected variable path other
  if [[ ! -r "$workspace/install/setup.bash" ]]; then
    echo "Missing underlay setup: $workspace/install/setup.bash" >&2; return 1
  fi
  detected=$(env -i HOME="$HOME" PATH=/usr/bin:/bin bash --noprofile --norc -c '
    set -e
    source "$1/install/setup.bash" >&2
    printf "%s" "${ROS_DISTRO:-}"
  ' _ "$workspace") || return 1
  if [[ $detected != humble && $detected != jazzy ]]; then
    echo "Underlay ROS distribution is unsupported or undetectable: ${detected:-unset}" >&2; return 1
  fi
  if [[ $requested != auto && $requested != "$detected" ]]; then
    echo "Requested ROS $requested differs from underlay ROS $detected." >&2; return 1
  fi
  if [[ -n ${ROS_DISTRO:-} && $ROS_DISTRO != "$detected" ]]; then
    echo "Current shell ROS $ROS_DISTRO differs from underlay ROS $detected; use a fresh shell." >&2; return 1
  fi
  if [[ $detected == humble ]]; then other=jazzy; else other=humble; fi
  for variable in PATH PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH; do
    path=":${!variable:-}:"
    if [[ $path == *"/opt/ros/$other/"* || $path == *"/opt/ros/$other:"* ]]; then
      echo "Other ROS distribution found in $variable; use a fresh shell." >&2; return 1
    fi
  done
  if [[ ! -r /opt/ros/$detected/setup.bash ]]; then
    echo "Missing /opt/ros/$detected/setup.bash" >&2; return 1
  fi
  printf '%s\n' "$detected"
}
