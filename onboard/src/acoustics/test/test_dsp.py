"""Focused contract, timing, ambiguity, and offline replay checks without ROS."""
# ruff: noqa: PLR2004 - literal expected values document synthetic fixtures
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from acoustics.dsp.config import DSPConfig, load_config
from acoustics.dsp.contracts import PAIRS, ArrayGeometry, DelayCandidate, SampleBlock, Status, TDOAResult
from acoustics.dsp.detection import extract_window
from acoustics.dsp.localization import FarFieldLocalizer
from acoustics.dsp.pipeline import Pipeline
from acoustics.dsp.tdoa import enforce_physical_bounds
from acoustics.dsp.tracking import PendingBearingTracker, RobotPose
from scipy.signal import correlate, correlation_lags

CONFIG_PATH = Path(__file__).parents[1] / 'config' / 'dsp.yaml'


@pytest.fixture
def config() -> DSPConfig:
    """Load the shipped noncoplanar example configuration."""
    return replace(load_config(CONFIG_PATH), enable_nearby=False, margin_front_s=0.0, margin_end_s=0.0,
                   raw_signal_threshold=0.1, window_duration_s=0.0005)


def delays_for_direction(direction: np.ndarray, config: DSPConfig) -> tuple[TDOAResult, ...]:
    """Generate physical delays from a source-facing direction using the declared sign."""
    geometry = config.geometry
    arrivals = -geometry.coordinates_m @ direction / config.sound_speed_m_s
    return tuple(TDOAResult(
        (geometry.channel_ids[i], geometry.channel_ids[j]),
        (DelayCandidate(float(arrivals[i] - arrivals[j]), {'synthetic': 1.0}),),
        float(np.linalg.norm(geometry.coordinates_m[i] - geometry.coordinates_m[j]) / config.sound_speed_m_s),
        Status.OK,
    ) for i, j in PAIRS)


def tone_block(config: DSPConfig) -> SampleBlock:
    """Create an isolated well-conditioned four-channel burst in normalized units."""
    size = 4096
    burst = np.zeros(size)
    grid = np.arange(800)
    burst[1600:2400] = 0.5 * np.sin(2 * np.pi * config.pinger_frequency_hz * grid / config.sample_rate_hz)
    return SampleBlock(np.tile(burst[:, None], (1, 4)), config.geometry.channel_ids,
                       config.sample_rate_hz, 100.0, 'normalized', 5000)


def test_delay_sign_and_source_facing_direction(config: DSPConfig) -> None:
    """A +x source reaches the +x hydrophone earlier, and tau_01 is positive."""
    direction = np.array([1.0, 0.0, 0.0])
    pairs = delays_for_direction(direction, config)
    assert pairs[0].candidates[0].delay_s > 0
    result = FarFieldLocalizer().localize(pairs, 12.0, config)
    assert result.status == Status.OK
    np.testing.assert_allclose(result.direction_candidates[0].unit_vector, direction, atol=1e-12)
    assert result.direction_candidates[0].azimuth_rad == pytest.approx(0.0)


def test_common_window_preserves_lag_and_original_time(config: DSPConfig) -> None:
    """A common crop retains a known signed delay, acquisition timestamp, and indices."""
    samples = np.zeros((100, 4))
    samples[43, 0], samples[40, 1] = 1.0, 1.0
    block = SampleBlock(samples, config.geometry.channel_ids, 1000.0, 7.0, 'V', 2000)
    window = extract_window(block, 30, 60)
    correlation = correlate(window.samples[:, 0], window.samples[:, 1])
    lag = correlation_lags(len(window.samples), len(window.samples))[np.argmax(correlation)]
    assert lag == 3  # tau_01 > 0: channel 0 is later.
    assert window.timestamp_s == pytest.approx(7.03)
    assert window.first_sample_index == 2030
    np.testing.assert_array_equal(window.samples, samples[30:60])


def test_planar_mirrors_and_explicit_constraint(config: DSPConfig) -> None:
    """A planar array must preserve the unresolved normal sign unless explicitly constrained."""
    geometry = replace(config.geometry, coordinates_m=np.array([
        [0, 0, 0], [0.05, 0, 0], [0, 0.05, 0], [0.05, 0.05, 0],
    ]))
    configured = replace(config, geometry=geometry)
    direction = np.array([0.6, 0.0, 0.8])
    pairs = delays_for_direction(direction, configured)
    result = FarFieldLocalizer().localize(pairs, 0.0, configured)
    assert result.status == Status.AMBIGUOUS
    assert len(result.direction_candidates) == 2
    assert sorted(candidate.unit_vector[2] for candidate in result.direction_candidates) == pytest.approx([-0.8, 0.8])
    constrained = FarFieldLocalizer().localize(pairs, 0.0, replace(configured, planar_constraint='positive_normal'))
    assert constrained.status == Status.OK
    np.testing.assert_allclose(constrained.direction_candidates[0].unit_vector, direction, atol=1e-12)


def test_physical_bounds_and_redundant_pair_consistency(config: DSPConfig) -> None:
    """Impossible delays are filtered; mutually contradictory pairs fail localization."""
    pairs = delays_for_direction(np.array([1.0, 0, 0]), config)
    impossible = replace(pairs[0], candidates=(DelayCandidate(1.0),))
    bounded = enforce_physical_bounds((impossible, *pairs[1:]), config)
    assert bounded[0].status == Status.INVALID
    assert not bounded[0].candidates
    contradictory = replace(pairs[-1], candidates=(DelayCandidate(-pairs[-1].physical_bound_s),))
    result = FarFieldLocalizer().localize((*pairs[:-1], contradictory), 0.0, config)
    assert result.status == Status.INVALID


def test_cycle_candidates_are_not_collapsed_to_one_peak(config: DSPConfig) -> None:
    """Valid candidate combinations retain competing bearings rather than an arbitrary winner."""
    forward = delays_for_direction(np.array([1.0, 0, 0]), config)
    backward = delays_for_direction(np.array([-1.0, 0, 0]), config)
    alternatives = tuple(replace(a, candidates=a.candidates + b.candidates)
                         for a, b in zip(forward, backward, strict=True))
    result = FarFieldLocalizer().localize(alternatives, 0.0, config)
    assert result.status == Status.AMBIGUOUS
    assert len(result.direction_candidates) >= 2
    limited = FarFieldLocalizer().localize(alternatives, 0.0, replace(config, max_candidate_combinations=1))
    assert limited.status == Status.AMBIGUOUS
    assert limited.ambiguity == 'candidate_limit'
    assert not limited.direction_candidates


def test_pipeline_reports_unfinished_delays(config: DSPConfig) -> None:
    """A detected burst cannot become a successful bearing through placeholder estimates."""
    result = Pipeline(config).process(tone_block(config))
    assert result.detections
    assert result.status == Status.NOT_IMPLEMENTED
    assert all(len(pairs) == 6 for pairs in result.delays)
    assert all(pair.status == Status.NOT_IMPLEMENTED and not pair.candidates
               for pairs in result.delays for pair in pairs)
    assert all(not result.direction_candidates for result in result.localizations)


def test_detector_and_pipeline_reject_bad_signal(config: DSPConfig) -> None:
    """Clipping, weak channels, silence, and truncated windows produce explicit outcomes."""
    block = tone_block(config)
    clipped = block.samples.copy()
    clipped[0, 0] = config.clipping_level
    clipped_result = Pipeline(replace(config, clipping_enabled=True)).process(replace(block, samples=clipped))
    assert clipped_result.status == Status.INVALID
    assert Pipeline(config).process(replace(block, samples=np.zeros_like(block.samples))).status == Status.NO_DETECTION
    weak = block.samples.copy()
    weak[:, 3] = 0
    assert Pipeline(config).process(replace(block, samples=weak)).status == Status.INVALID
    truncated = Pipeline(config).process(extract_window(block, 1700, 2350))
    assert truncated.detections
    assert truncated.status == Status.INVALID


def test_tracker_does_not_invent_range(config: DSPConfig) -> None:
    """Future multi-view tracking validates timestamps and never reports an invented position."""
    bearing = FarFieldLocalizer().localize(delays_for_direction(np.array([1.0, 0, 0]), config), 4.0, config)
    pose = RobotPose(4.0, 'map', config.geometry.frame_id, np.zeros(3), np.eye(3))
    result = PendingBearingTracker().update(bearing, pose)
    assert result.status == Status.NOT_IMPLEMENTED
    assert result.position_m is None
    assert PendingBearingTracker().update(bearing, replace(pose, timestamp_s=5.0)).status == Status.INVALID


def test_invalid_configuration_and_acquisition(config: DSPConfig) -> None:
    """Invalid rates, geometry, channel IDs, and short upstream filter blocks fail explicitly."""
    with pytest.raises(ValueError, match='Nyquist'):
        replace(config, sample_rate_hz=40000)
    with pytest.raises(ValueError, match='four unique'):
        ArrayGeometry(('a', 'a', 'b', 'c'), np.zeros((4, 3)), 'array', np.zeros(4))
    block = tone_block(config)
    with pytest.raises(ValueError, match='sample rates differ'):
        Pipeline(config).process(replace(block, sample_rate_hz=48000.0))
    with pytest.raises(ValueError, match='padlen'):
        Pipeline(config).process(extract_window(block, 0, 4))
