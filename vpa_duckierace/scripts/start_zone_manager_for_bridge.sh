#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
unset ROS_HOSTNAME
export PATH="/usr/bin:/bin:$PATH"

exec roslaunch vpa_duckierace zone_manager.launch
