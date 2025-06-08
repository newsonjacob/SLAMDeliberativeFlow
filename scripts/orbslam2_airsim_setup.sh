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
Setup complete. Add the following to your environment:

source $WS/devel/setup.bash

Then launch AirSim and run:

rosrun slam_deliberative_flow airsim_image_publisher.py
roslaunch orb_slam2_ros mono.launch camera_topic:=/airsim/image_raw
EOM
