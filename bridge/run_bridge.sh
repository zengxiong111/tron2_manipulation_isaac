#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 BRIDGE_JAZZY_INSTALL" >&2
  exit 2
fi

bridge_install=$(realpath "$1")
ros_setup=/opt/ros/jazzy/setup.bash
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Jazzy setup not found: $ros_setup" >&2
  exit 1
fi
if [[ ! -f "$bridge_install/setup.bash" ]]; then
  echo "ros2-bridger setup not found: $bridge_install/setup.bash" >&2
  exit 1
fi

# ROS 2 setup scripts read unset variables in nounset mode.
set +u
source "$ros_setup"
source "$bridge_install/setup.bash"
set -u

# The port-5000/MuJoCo acceptance path runs in a private loopback namespace.
export MROS_IP_LIST=127.0.0.1
export MROS_LOCALHOST_ONLY=0

exec ros2 launch mrosbridger mrosbridger.launch.py \
  bridge_ros2mros:=true \
  mros2ros_include:=/motor/cmd \
  ros2mros_include:=/motor/state
