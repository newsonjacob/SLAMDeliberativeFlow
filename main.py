"""Example main script performing waypoint navigation using SLAM poses."""
import time
import math
import os
import yaml
import airsim
import subprocess

from slam.slam_interface import get_current_pose
from navigation.deliberative_nav import compute_velocity_command, compute_yaw_command
from uav.airsim_utils import connect_and_takeoff
from uav.interface import start_gui, exit_flag, exit_event


# Path to the AirSim Unreal executable. Can be overridden using the
# ``UE4_PATH`` environment variable.
UE4_EXE_PATH = os.environ.get("UE4_PATH", r"C:\Users\newso\Documents\AirSimExperiments\BlocksBuild\WindowsNoEditor\Blocks\Binaries\Win64\Blocks.exe")
ue4_proc = None
slam_proc = None


def _reached_goal(current: tuple, goal: tuple, threshold: float = 1.0) -> bool:
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    dz = goal[2] - current[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz) < threshold


def run_navigation() -> None:
    global ue4_proc, slam_proc

    print("\U0001f7e6 Launching Unreal simulation...")
    ue4_proc = subprocess.Popen([
        UE4_EXE_PATH,
        "-windowed",
        "-ResX=1280",
        "-ResY=720"
    ])


    time.sleep(5)

    print("\U0001f7e7 Starting simulated SLAM writer...")
    slam_proc = subprocess.Popen(["python", "simulated_slam_writer.py"])
    time.sleep(1)

    client = airsim.MultirotorClient()
    connect_and_takeoff(client, altitude=-2.0)

    with open("config/goals.yaml", "r") as f:
        goals = yaml.safe_load(f)

    flight_log = []

    for goal_index, goal in enumerate(goals):
        goal_pose = tuple(goal)

        while not exit_flag[0]:
            current_pose = get_current_pose()
            assert len(current_pose) == 4 and isinstance(current_pose, tuple)

            dx = goal_pose[0] - current_pose[0]
            dy = goal_pose[1] - current_pose[1]
            dz = goal_pose[2] - current_pose[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            print(f"[{goal_index}] Distance to goal: {dist:.2f} | Current: {current_pose} | Goal: {goal_pose}")

            if _reached_goal(current_pose, goal_pose):
                print(f"✅ Reached waypoint {goal_index}")
                break

            yaw_error = math.atan2(goal_pose[1] - current_pose[1],
                                goal_pose[0] - current_pose[0]) - current_pose[3]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            yaw_rate = compute_yaw_command(current_pose[3],
                                        current_pose[:3],
                                        goal_pose[:3])

            flight_log.append((time.time(), *current_pose, yaw_error, yaw_rate))

            vx, vy, vz = compute_velocity_command(current_pose, goal_pose)
            print(f"Commanded velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

            try:
                client.moveByVelocityAsync(
                    vx, vy, vz, 1.0,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)
                ).join()
            except Exception as e:
                print(f"⚠️ Velocity command failed: {e}")
                break

            time.sleep(0.1)

        if exit_flag[0]:
            break


    if exit_flag[0]:
            print("\U0001f6d1 Stop requested \u2014 landing.")
    else:
        print("All waypoints reached \u2014 landing.")

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

    os.makedirs("logs", exist_ok=True)
    with open("logs/flight_log.csv", "w") as f:
        f.write("time,x,y,z,yaw,yaw_error,yaw_rate\n")
        for entry in flight_log:
            f.write(",".join(f"{v:.3f}" for v in entry) + "\n")

    # Kill SLAM writer and UE4 sim
    if slam_proc:
        slam_proc.kill()
    if ue4_proc:
        ue4_proc.kill()

    print("🛑 SLAM writer and UE4 simulation terminated.")

    # Trigger GUI shutdown
    exit_event.set()




def main() -> None:
    import threading
    nav_thread = threading.Thread(target=run_navigation)
    nav_thread.start()

    # GUI must run in the main thread on Windows
    start_gui()



if __name__ == "__main__":
    main()
