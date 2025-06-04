#!/usr/bin/env bash
# Script to set up ORB-SLAM2 ROS for use with AirSim

set -e

WS="$HOME/orbslam2_ws"
mkdir -p "$WS/src"
cd "$WS/src"
if [ ! -d "orb_slam_2_ros" ]; then
  git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
fi

cd "$WS"
catkin_make

cat <<EOM
\nSetup complete. Add the following to your environment:\n\nsource $WS/devel/setup.bash\n\nThen launch AirSim and run:\n\nrosrun slam_deliberative_flow airsim_image_publisher.py\nroslaunch orb_slam2_ros mono.launch camera_topic:=/airsim/image_raw\nEOM
