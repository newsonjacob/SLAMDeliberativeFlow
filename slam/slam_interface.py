"""Stub interface for getting drone pose from SLAM."""
from typing import Tuple
import time


def get_current_pose() -> Tuple[float, float, float, float]:
    """Return the drone's current pose as ``(x, y, z, yaw)``.

    This is a stub implementation that generates a dummy pose that
    changes over time. Real SLAM integration will replace this
    logic in a later phase.
    """
    t = time.time()
    x = 1.0 + 0.1 * (t % 50)  # slowly increasing x position
    y = 2.0
    z = -2.0
    yaw = 0.0
    return x, y, z, yaw
