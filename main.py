"""Example main script performing waypoint navigation using SLAM poses."""
import time
import math
import yaml
import airsim

from slam.slam_interface import get_current_pose
from navigation.deliberative_nav import compute_velocity_command
from uav.airsim_utils import connect_and_takeoff, send_velocity_command


def _reached_goal(current: tuple, goal: tuple, threshold: float = 0.5) -> bool:
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    dz = goal[2] - current[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz) < threshold


def main() -> None:
    client = airsim.MultirotorClient()
    connect_and_takeoff(client, altitude=-2.0)

    with open("config/goals.yaml", "r") as f:
        goals = yaml.safe_load(f)

    for goal in goals:
        goal_pose = tuple(goal)
        while True:
            current_pose = get_current_pose()
            if _reached_goal(current_pose, goal_pose):
                break
            vx, vy, vz = compute_velocity_command(current_pose, goal_pose)
            send_velocity_command(client, vx, vy, vz, duration=1.0)
            time.sleep(0.1)

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)


if __name__ == "__main__":
    main()
