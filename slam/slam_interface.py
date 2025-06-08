"""Interface for retrieving the drone pose.

The primary source of pose data is an external SLAM process which writes the
latest estimate to ``slam/latest_pose.txt``. If the file cannot be read or
contains stale data, the code falls back to AirSim's ground-truth pose when
available. A short rolling average is applied to smooth pose estimates.
"""
from typing import Deque, Optional, Tuple
from collections import deque
import logging
import os
import time
import socket

import airsim


_CLIENT: Optional[airsim.MultirotorClient]
try:
    _CLIENT = airsim.MultirotorClient()
    _CLIENT.confirmConnection()
except Exception as exc:  # pragma: no cover - connection failure is non-fatal
    logging.warning("Failed to connect to AirSim for fallback poses: %s", exc)
    _CLIENT = None

# Keep a short history to optionally smooth pose estimates
_POSE_BUFFER: Deque[Tuple[float, float, float, float]] = deque(maxlen=5)

# path to the shared pose file written by the external SLAM system
_POSE_FILE = os.path.join(os.path.dirname(__file__), 'latest_pose.txt')

# ignore pose entries older than this many seconds
_POSE_TIMEOUT = 1.0


def _average_pose() -> Tuple[float, float, float, float]:
    """Return the mean pose from the buffer."""
    n = len(_POSE_BUFFER)
    sx = sum(p[0] for p in _POSE_BUFFER)
    sy = sum(p[1] for p in _POSE_BUFFER)
    sz = sum(p[2] for p in _POSE_BUFFER)
    syaw = sum(p[3] for p in _POSE_BUFFER)
    return sx / n, sy / n, sz / n, syaw / n


def read_pose_from_file() -> Optional[Tuple[float, float, float, float]]:
    """Parse the latest pose from ``_POSE_FILE``.

    The file is expected to contain four or five whitespace or comma separated
    values ``x y z yaw [timestamp]``.  If a timestamp is provided and the entry
    is older than ``_POSE_TIMEOUT`` seconds it will be ignored.
    """

    try:
        with open(_POSE_FILE, 'r') as f:
            line = f.readline().strip()
    except FileNotFoundError:
        logging.warning("SLAM pose file '%s' not found", _POSE_FILE)
        return None
    except OSError as exc:
        logging.warning("Failed to read SLAM pose file: %s", exc)
        return None

    if not line:
        return None

    parts = [p for p in line.replace(',', ' ').split() if p]
    if len(parts) < 4:
        logging.warning("Malformed SLAM pose entry: '%s'", line)
        return None

    try:
        x, y, z, yaw = map(float, parts[:4])
    except ValueError:
        logging.warning("Failed to parse SLAM pose: '%s'", line)
        return None

    if len(parts) >= 5:
        try:
            ts = float(parts[4])
            if time.time() - ts > _POSE_TIMEOUT:
                logging.warning("Ignoring stale SLAM pose (%.2fs old)", time.time() - ts)
                return None
        except ValueError:
            logging.warning("Invalid timestamp in SLAM pose: '%s'", line)

    return x, y, z, yaw


def connect_to_slam_socket(host: str = '127.0.0.1', port: int = 30002) -> Optional[socket.socket]:
    """Establish a non-blocking connection to a SLAM TCP socket."""
    try:
        sock = socket.create_connection((host, port), timeout=0.5)
        sock.setblocking(False)
        return sock
    except OSError as exc:
        logging.warning("Failed to connect to SLAM socket %s:%s - %s", host, port, exc)
        return None


def get_current_pose() -> Tuple[float, float, float, float]:
    """Return the drone's current pose as ``(x, y, z, yaw)``.

    The function first attempts to obtain the latest pose estimate from the
    external SLAM system via ``read_pose_from_file``.  If this fails and an
    AirSim connection is available, the simulator's ground-truth pose is used as
    a fallback.  Should both sources be unavailable, ``(0, 0, 0, 0)`` is
    returned.  A short moving average smooths consecutive poses.
    """

    pose = read_pose_from_file()

    if pose is None and _CLIENT is not None:
        try:
            sim_pose = _CLIENT.simGetVehiclePose()
            pose = (
                sim_pose.position.x_val,
                sim_pose.position.y_val,
                sim_pose.position.z_val,
                airsim.to_eularian_angles(sim_pose.orientation)[2],
            )
        except Exception as exc:  # pragma: no cover - unexpected AirSim error
            logging.warning("Failed to retrieve AirSim pose: %s", exc)
            pose = None

    if pose is None:
        pose = (0.0, 0.0, 0.0, 0.0)

    _POSE_BUFFER.append(pose)
    return _average_pose() if len(_POSE_BUFFER) > 1 else pose
