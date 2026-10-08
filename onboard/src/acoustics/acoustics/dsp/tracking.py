"""Interfaces for position estimation from bearings and timestamped robot poses."""
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from acoustics.dsp.contracts import LocalizationResult, Status

POSE_TIMESTAMP_TOLERANCE_S = 1e-6


@dataclass(frozen=True)
class RobotPose:
    """
    Pose at the bearing timestamp, mapping the array frame into a world frame.

    Rotation is a 3x3 orthonormal matrix, translation is meters. A future ROS adapter
    must interpolate poses and apply the hydrophone-array extrinsic transform.
    """

    timestamp_s: float
    world_frame_id: str
    array_frame_id: str
    translation_m: NDArray[np.float64]
    rotation_world_from_array: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Reject invalid transforms before they enter a tracker."""
        rotation = self.rotation_world_from_array
        if self.translation_m.shape != (3,) or rotation.shape != (3, 3):
            msg = 'pose must contain a 3-vector and a 3x3 rotation'
            raise ValueError(msg)
        if not np.isfinite(self.translation_m).all() or not np.isfinite(rotation).all():
            msg = 'pose values must be finite'
            raise ValueError(msg)
        if not np.allclose(rotation.T @ rotation, np.eye(3)) or not np.isclose(np.linalg.det(rotation), 1):
            msg = 'pose rotation must be a proper orthonormal matrix'
            raise ValueError(msg)
        if not np.isfinite(self.timestamp_s) or not self.world_frame_id or not self.array_frame_id:
            msg = 'pose requires a finite timestamp and coordinate frames'
            raise ValueError(msg)


class BearingTracker(Protocol):
    """Extension point for triangulation, robot motion, and observability checks."""

    def update(self, bearing: LocalizationResult, pose: RobotPose) -> LocalizationResult:
        """Combine an observation with its time-aligned array pose."""
        ...


class PendingBearingTracker:
    """Position tracking placeholder; never returns a fabricated source position."""

    def update(self, bearing: LocalizationResult, pose: RobotPose) -> LocalizationResult:
        """Validate association and explicitly report unfinished triangulation."""
        if (bearing.frame_id != pose.array_frame_id
                or abs(bearing.timestamp_s - pose.timestamp_s) > POSE_TIMESTAMP_TOLERANCE_S):
            return LocalizationResult(bearing.timestamp_s, pose.world_frame_id, Status.INVALID,
                                      reason='bearing and pose must have matching frame and timestamp')
        return LocalizationResult(bearing.timestamp_s, pose.world_frame_id, Status.NOT_IMPLEMENTED,
                                  reason='TODO: multi-bearing triangulation, observability, and uncertainty')
