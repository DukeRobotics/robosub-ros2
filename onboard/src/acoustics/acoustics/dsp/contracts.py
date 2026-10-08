"""Data contracts shared by acquisition, DSP, replay, and ROS adapters."""
from dataclasses import dataclass, field
from enum import StrEnum
from itertools import combinations

import numpy as np
from numpy.typing import NDArray

CHANNEL_COUNT = 4
SAMPLE_MATRIX_NDIM = 2
PAIRS = tuple(combinations(range(CHANNEL_COUNT), 2))


class Status(StrEnum):
    """Explicit processing outcomes; only OK represents a usable estimate."""

    OK = 'ok'
    NO_DETECTION = 'no_detection'
    INVALID = 'invalid'
    AMBIGUOUS = 'ambiguous'
    NOT_IMPLEMENTED = 'not_implemented'


@dataclass(frozen=True)
class SampleBlock:
    """
    Synchronized samples [sample, channel]; timestamp is first sample, in seconds.

    first_sample_index is the first row's index in the original acquisition.
    Channel IDs identify hardware channels, not their positions in the input array.
    """

    samples: NDArray[np.float64]
    channel_ids: tuple[str, ...]
    sample_rate_hz: float
    timestamp_s: float
    units: str
    first_sample_index: int = 0
    timestamp_reference: str = 'acquisition'

    def __post_init__(self) -> None:
        """Reject malformed or unsynchronized input contracts."""
        if self.samples.ndim != SAMPLE_MATRIX_NDIM or self.samples.shape[1] != CHANNEL_COUNT or len(self.samples) == 0:
            msg = 'samples must have shape (N, 4), with N > 0'
            raise ValueError(msg)
        if len(self.channel_ids) != CHANNEL_COUNT or len(set(self.channel_ids)) != CHANNEL_COUNT:
            msg = 'four unique channel IDs are required'
            raise ValueError(msg)
        if not np.isfinite(self.samples).all():
            msg = 'samples must be finite'
            raise ValueError(msg)
        if not np.isfinite(self.sample_rate_hz) or self.sample_rate_hz <= 0:
            msg = 'sample rate must be positive and finite'
            raise ValueError(msg)
        if not np.isfinite(self.timestamp_s) or not self.units or self.first_sample_index < 0:
            msg = 'invalid timestamp, units, or original sample index'
            raise ValueError(msg)


@dataclass(frozen=True)
class ArrayGeometry:
    """Coordinates follow channel_ids; positive offset means that channel records late."""

    channel_ids: tuple[str, ...]
    coordinates_m: NDArray[np.float64]
    frame_id: str
    timing_offsets_s: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate geometry and calibration without assuming a planar layout."""
        if len(self.channel_ids) != CHANNEL_COUNT or len(set(self.channel_ids)) != CHANNEL_COUNT:
            msg = 'geometry requires four unique channel IDs'
            raise ValueError(msg)
        if self.coordinates_m.shape != (CHANNEL_COUNT, 3) or self.timing_offsets_s.shape != (CHANNEL_COUNT,):
            msg = 'geometry must be (4, 3), calibration must be (4,)'
            raise ValueError(msg)
        if not np.isfinite(self.coordinates_m).all() or not np.isfinite(self.timing_offsets_s).all():
            msg = 'geometry and calibration must be finite'
            raise ValueError(msg)
        if not self.frame_id or any(
            np.linalg.norm(self.coordinates_m[i] - self.coordinates_m[j]) == 0 for i, j in PAIRS
        ):
            msg = 'a frame and four distinct hydrophone positions are required'
            raise ValueError(msg)


@dataclass(frozen=True)
class PingDetection:
    """
    One common waveform window; original start/end indices use an exclusive end.

    onset_sample_index identifies a common trigger on the acquisition grid. Individual
    channels must NEVER be independently shifted or cropped around their own onset.
    Timing calibration affects TOA/delay metadata, not the waveform's sample grid.
    """

    window: SampleBlock
    start_sample_index: int
    end_sample_index: int
    onset_sample_index: int
    frequency_hz: float | None
    quality: dict[str, float] = field(default_factory=dict)
    status: Status = Status.OK
    reason: str = ''


@dataclass(frozen=True)
class DelayCandidate:
    """A plausible tau_ij = t_i - t_j, in seconds, with estimator quality metrics."""

    delay_s: float
    quality: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class TDOAResult:
    """All plausible delays for one ordered pair, never an implicit zero fallback."""

    channel_pair: tuple[str, str]
    candidates: tuple[DelayCandidate, ...]
    physical_bound_s: float
    status: Status
    reason: str = ''


@dataclass(frozen=True)
class DirectionCandidate:
    """Unit vector points from the array toward the source; angles are radians."""

    unit_vector: tuple[float, float, float]
    azimuth_rad: float
    elevation_rad: float
    residual_rms_m: float


@dataclass(frozen=True)
class LocalizationResult:
    """
    Bearing candidates and optional future position/covariance estimate.

    Ambiguity may arise from array geometry or multiple tone-cycle delay candidates.
    uncertainty is optional: no covariance is better than fabricated confidence.
    """

    timestamp_s: float
    frame_id: str
    status: Status
    direction_candidates: tuple[DirectionCandidate, ...] = ()
    position_m: tuple[float, float, float] | None = None
    uncertainty: NDArray[np.float64] | None = None
    ambiguity: str = 'none'
    reason: str = ''


@dataclass(frozen=True)
class ProcessingResult:
    """Block-level outcome with a result for every detected ping."""

    status: Status
    detections: tuple[PingDetection, ...] = ()
    delays: tuple[tuple[TDOAResult, ...], ...] = ()
    localizations: tuple[LocalizationResult, ...] = ()
    reason: str = ''
    channel_analysis: tuple['HydrophoneAnalysis', ...] = ()


@dataclass(frozen=True)
class HydrophoneAnalysis:
    """Upstream per-channel analysis with raw and offset-calibrated arrival times."""

    channel_id: str
    toa_sample_index: int
    raw_toa_s: float
    calibrated_toa_s: float
    valid: bool
    validation_reason: str
    nearby: bool | None = None
    nearby_confidence: float | None = None
