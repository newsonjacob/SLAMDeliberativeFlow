"""Simple interface for retrieving the drone pose from a SLAM file."""
from __future__ import annotations

import os
from typing import Tuple


# path to the pose file written by ``simulated_slam_writer.py``
POSE_FILE = os.path.join(os.path.dirname(__file__), "latest_pose.txt")

# fallback pose if the file is missing or cannot be parsed
_FALLBACK_POSE = (0.0, 0.0, -2.0, 0.0)


def get_current_pose() -> Tuple[float, float, float, float]:
    """Return the current drone pose ``(x, y, z, yaw)``.

    The pose is read from :data:`POSE_FILE` which must contain a single line in
    ``x,y,z,yaw`` format.  If the file is missing or cannot be parsed a fallback
    pose is returned.
    """
    if not os.path.exists(POSE_FILE):
        return _FALLBACK_POSE

    try:
        with open(POSE_FILE, "r") as f:
            line = f.readline()
            parts = line.strip().split(",")
            x, y, z, yaw = map(float, parts[:4])
            return x, y, z, yaw
    except Exception:
        return _FALLBACK_POSE
