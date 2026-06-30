#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
unset ROS_HOSTNAME
export PATH="/usr/bin:/bin:$PATH"

if ! rosnode list >/dev/null 2>&1; then
  echo "roscore is not reachable at ${ROS_MASTER_URI}" >&2
  exit 1
fi

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

rosrun vpa_duckierace tcp_jpeg_image_publisher.py \
  __name:=tcp_cam1 \
  _port:=5010 \
  _topic:=/usb_cam_1/image_raw \
  _frame_id:=cam1 &
pids+=("$!")

rosrun vpa_duckierace tcp_jpeg_image_publisher.py \
  __name:=tcp_cam2 \
  _port:=5011 \
  _topic:=/usb_cam_2/image_raw \
  _frame_id:=cam2 &
pids+=("$!")

echo "ROS camera bridge started:"
echo "  /usb_cam_1/image_raw <- TCP 5010"
echo "  /usb_cam_2/image_raw <- TCP 5011"
echo "Press Ctrl-C to stop."

wait
