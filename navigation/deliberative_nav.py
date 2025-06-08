"""Simple deliberative navigation helpers."""
from typing import Tuple


# proportional gain used for velocity commands
_KP = 0.5


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
