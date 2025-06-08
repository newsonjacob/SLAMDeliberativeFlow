"""Utility helpers for controlling an AirSim multirotor."""
from typing import Tuple
import airsim


def connect_and_takeoff(client: airsim.MultirotorClient, altitude: float = -2.0) -> None:
    """Connect to AirSim and take off to the specified altitude."""
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(altitude, 1).join()


def send_velocity_command(client: airsim.MultirotorClient, vx: float, vy: float, vz: float, duration: float = 1.0) -> None:
    """Send a velocity command to the drone for ``duration`` seconds."""
    client.moveByVelocityAsync(vx, vy, vz, duration,
                               drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                               yaw_mode=airsim.YawMode(False, 0)).join()
