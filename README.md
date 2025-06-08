# SLAM Deliberative Flow

This repository demonstrates how to combine ORB-SLAM2 with the AirSim simulator for basic obstacle avoidance experiments.

The setup relies on the [`orb_slam_2_ros`](https://github.com/appliedAI-Initiative/orb_slam_2_ros) wrapper which provides ROS nodes for ORB-SLAM2.

## Quick start

1. Install [ROS](https://www.ros.org) (tested with ROS Noetic) and [AirSim](https://github.com/microsoft/AirSim).
2. Run `scripts/orbslam2_airsim_setup.sh` to clone and build `orb_slam_2_ros` in a new workspace.
3. Start the AirSim simulator.
4. In a new terminal, source the workspace and launch the image publisher:
   ```bash
   source ~/orbslam2_ws/devel/setup.bash
   rosrun slam_deliberative_flow airsim_image_publisher.py
   ```
5. Launch ORB-SLAM2 with images streamed from AirSim:
   ```bash
   roslaunch orb_slam2_ros mono.launch camera_topic:=/airsim/image_raw
   ```

ORB-SLAM2 will provide pose estimates that can be used by a motion planner for obstacle avoidance.

## Files

- `scripts/airsim_image_publisher.py` – publishes AirSim camera images to ROS.
- `scripts/orbslam2_airsim_setup.sh` – helper to set up the `orb_slam_2_ros` workspace.

These scripts offer a starting point for integrating ORB-SLAM2-based navigation into an AirSim environment.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
