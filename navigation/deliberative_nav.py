"""Simple deliberative navigation helpers."""
from typing import Tuple
import math


# proportional gain used for velocity commands
_KP = 0.5

# proportional gain for yaw control
_KP_YAW = 1.0


def compute_velocity_command(current_pose: Tuple[float, float, float, float],
                             goal_pose: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """Compute a velocity vector ``(vx, vy, vz)`` moving the drone toward ``goal_pose``.

    The yaw component of the pose is ignored for now.
    """
    cx, cy, cz, _ = current_pose
    gx, gy, gz, _ = goal_pose

    vx = _KP * (gx - cx)
    vy = _KP * (gy - cy)
    vz = _KP * (gz - cz)
    return vx, vy, vz


def compute_yaw_command(current_yaw: float, current_pos: Tuple[float, float, float],
                        goal_pos: Tuple[float, float, float]) -> float:
    """Compute a yaw rate command rotating the drone toward ``goal_pos``.

    Parameters
    ----------
    current_yaw : float
        Current yaw of the drone in radians.
    current_pos : Tuple[float, float, float]
        Current position ``(x, y, z)`` of the drone.
    goal_pos : Tuple[float, float, float]
        Desired goal position ``(x, y, z)``.

    Returns
    -------
    float
        Yaw rate command in radians per second, limited to ``\u00b10.5``.
    """

    dx = goal_pos[0] - current_pos[0]
    dy = goal_pos[1] - current_pos[1]

    desired_yaw = math.atan2(dy, dx)
    yaw_error = desired_yaw - current_yaw

    # normalize error to [-pi, pi]
    yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

    yaw_rate = _KP_YAW * yaw_error

    # limit yaw rate to +/-0.5 rad/s
    yaw_rate = max(-0.5, min(0.5, yaw_rate))
    return yaw_rate
