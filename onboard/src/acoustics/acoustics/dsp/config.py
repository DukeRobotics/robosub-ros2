"""Validated configuration usable from ROS or an offline YAML file."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from acoustics.dsp.contracts import ArrayGeometry


@dataclass(frozen=True)
class DSPConfig:
    """Processing settings; geometry is measured in the declared robot-relative frame."""

    geometry: ArrayGeometry
    sample_rate_hz: float = 781250.0
    pinger_frequency_hz: float = 32000.0
    filter_bandwidth_hz: float = 4000.0
    filter_order: int = 6
    sound_speed_m_s: float = 1482.0
    pre_trigger_s: float = 0.001
    post_trigger_s: float = 0.001
    clipping_level: float = 1.0
    pair_consistency_tolerance_s: float = 0.00001
    fit_tolerance_m: float = 0.005
    max_candidate_combinations: int = 256
    planar_constraint: str = 'none'
    clipping_enabled: bool = False
    toa_threshold_sigma: float = 5.0
    raw_signal_threshold: float = 0.5
    margin_front_s: float = 0.1
    margin_end_s: float = 0.1
    window_duration_s: float = 0.01
    capture_duration_s: float = 2.0
    capture_output_dir: str = 'Temp_Data'
    use_mock_device: bool = False
    enable_nearby: bool = True
    nearby_model_path: str = ''

    def __post_init__(self) -> None:
        """Reject settings that cannot define a physical processing pipeline."""
        positive = (
            self.sample_rate_hz, self.pinger_frequency_hz, self.filter_bandwidth_hz,
            self.sound_speed_m_s,
            self.clipping_level, self.pair_consistency_tolerance_s, self.fit_tolerance_m,
            self.toa_threshold_sigma, self.raw_signal_threshold, self.window_duration_s, self.capture_duration_s,
        )
        if any(not np.isfinite(value) or value <= 0 for value in positive):
            msg = 'rates, thresholds, durations, and tolerances must be positive and finite'
            raise ValueError(msg)
        if any(not np.isfinite(value) or value < 0 for value in (
            self.pre_trigger_s, self.post_trigger_s, self.margin_front_s, self.margin_end_s,
        )):
            msg = 'window durations and arrival margins must be finite and nonnegative'
            raise ValueError(msg)
        if not self.capture_output_dir:
            msg = 'capture_output_dir must not be empty'
            raise ValueError(msg)
        if not float(self.sample_rate_hz).is_integer():
            msg = 'Saleae analog_sample_rate must be an integer number of samples per second'
            raise ValueError(msg)
        if not isinstance(self.filter_order, int) or self.filter_order < 1:
            msg = 'filter_order must be a positive integer'
            raise ValueError(msg)
        if not isinstance(self.max_candidate_combinations, int) or self.max_candidate_combinations < 1:
            msg = 'max_candidate_combinations must be a positive integer'
            raise ValueError(msg)
        low = self.pinger_frequency_hz - self.filter_bandwidth_hz / 2
        high = self.pinger_frequency_hz + self.filter_bandwidth_hz / 2
        if not 0 < low < high < self.sample_rate_hz / 2:
            msg = 'band-pass must lie strictly between DC and Nyquist'
            raise ValueError(msg)
        if self.planar_constraint not in {'none', 'positive_normal', 'negative_normal'}:
            msg = 'unknown planar_constraint'
            raise ValueError(msg)


def load_config(path: str | Path) -> DSPConfig:
    """Load a plain DSP YAML configuration, independent of ROS parameter syntax."""
    with Path(path).open(encoding='utf-8') as stream:
        values = yaml.safe_load(stream)
    geometry = values.pop('geometry')
    return DSPConfig(
        geometry=ArrayGeometry(
            channel_ids=tuple(geometry['channel_ids']),
            coordinates_m=np.asarray(geometry['coordinates_m'], dtype=float),
            frame_id=geometry['frame_id'],
            timing_offsets_s=np.asarray(geometry['timing_offsets_s'], dtype=float),
        ),
        **values,
    )
