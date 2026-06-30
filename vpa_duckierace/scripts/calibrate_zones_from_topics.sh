#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi
set -u

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
unset ROS_HOSTNAME
export PATH="/usr/bin:/bin:$PATH"

pkg_dir="$(rospack find vpa_duckierace)"
cd "$pkg_dir"

mkdir -p test config
stamp="$(date +%Y%m%d_%H%M%S)"

if [ -f test/image1.png ]; then
  cp test/image1.png "test/image1.png.bak_${stamp}"
fi
if [ -f test/image2.png ]; then
  cp test/image2.png "test/image2.png.bak_${stamp}"
fi
if [ -f config/zones_cam1.yaml ]; then
  cp config/zones_cam1.yaml "config/zones_cam1.yaml.bak_${stamp}"
fi
if [ -f config/zones_cam2.yaml ]; then
  cp config/zones_cam2.yaml "config/zones_cam2.yaml.bak_${stamp}"
fi

echo "Capturing /usb_cam_1/image_raw -> test/image1.png"
timeout 8s rosrun image_view image_saver image:=/usb_cam_1/image_raw _filename_format:=test/image1.png >/tmp/cam1_image_saver.log 2>&1 || true
if [ ! -s test/image1.png ]; then
  cat /tmp/cam1_image_saver.log >&2
  exit 1
fi

echo "Capturing /usb_cam_2/image_raw -> test/image2.png"
timeout 8s rosrun image_view image_saver image:=/usb_cam_2/image_raw _filename_format:=test/image2.png >/tmp/cam2_image_saver.log 2>&1 || true
if [ ! -s test/image2.png ]; then
  cat /tmp/cam2_image_saver.log >&2
  exit 1
fi

echo "Running zone calibration"
/usr/bin/python3 scripts/auto_zone_cali.py

echo "Wrote:"
echo "  $pkg_dir/config/zones_cam1.yaml"
echo "  $pkg_dir/config/zones_cam2.yaml"
