import airsim
import time
import os
import argparse


def main(rate: float) -> None:
    client = airsim.MultirotorClient()
    client.confirmConnection()

    os.makedirs("slam", exist_ok=True)
    interval = 1.0 / rate if rate > 0 else 0.05

    while True:
        pose = client.simGetVehiclePose()
        x = pose.position.x_val
        y = pose.position.y_val
        z = pose.position.z_val
        _, _, yaw = airsim.to_eularian_angles(pose.orientation)
        timestamp = time.time()
        with open("slam/latest_pose.txt", "w") as f:
            f.write(f"{x:.3f},{y:.3f},{z:.3f},{yaw:.3f},{timestamp:.3f}")
        time.sleep(interval)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Write AirSim ground-truth pose to a file to simulate SLAM"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="update rate in Hz (default: 20)",
    )
    args = parser.parse_args()
    main(args.rate)
