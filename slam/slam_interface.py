"""Interface for retrieving the drone pose from AirSim."""
from typing import Deque, Tuple
from collections import deque
import airsim


_CLIENT = airsim.MultirotorClient()
try:
    _CLIENT.confirmConnection()
except Exception as exc:  # pragma: no cover - connection failure is fatal
    raise RuntimeError("Failed to connect to AirSim") from exc

# Keep a short history to optionally smooth pose estimates
_POSE_BUFFER: Deque[Tuple[float, float, float, float]] = deque(maxlen=5)


def _average_pose() -> Tuple[float, float, float, float]:
    """Return the mean pose from the buffer."""
    n = len(_POSE_BUFFER)
    sx = sum(p[0] for p in _POSE_BUFFER)
    sy = sum(p[1] for p in _POSE_BUFFER)
    sz = sum(p[2] for p in _POSE_BUFFER)
    syaw = sum(p[3] for p in _POSE_BUFFER)
    return sx / n, sy / n, sz / n, syaw / n


def get_current_pose() -> Tuple[float, float, float, float]:
    """Return the drone's current pose as ``(x, y, z, yaw)``.

    Pose values are obtained directly from AirSim in world coordinates
    (meters for position, radians for yaw). A simple moving average over
    the last few samples is applied to reduce jitter.
    """

    pose = _CLIENT.simGetVehiclePose()
    x = pose.position.x_val
    y = pose.position.y_val
    z = pose.position.z_val
    _, _, yaw = airsim.to_eularian_angles(pose.orientation)

    _POSE_BUFFER.append((x, y, z, yaw))
    if len(_POSE_BUFFER) > 1:
        return _average_pose()
    return x, y, z, yaw
