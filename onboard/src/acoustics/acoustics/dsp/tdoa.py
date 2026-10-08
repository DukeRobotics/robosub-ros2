"""Delay estimator interfaces; carrier-period alternatives must remain explicit."""
from dataclasses import replace
from typing import Protocol

import numpy as np

from acoustics.dsp.config import DSPConfig
from acoustics.dsp.contracts import PAIRS, PingDetection, Status, TDOAResult


class DelayEstimator(Protocol):
    """
    Estimate raw waveform delays for all six pairs using tau_ij = t_i - t_j.

    The pipeline applies electronic timing calibration once, before enforcing bounds.
    """

    def estimate(self, detection: PingDetection, config: DSPConfig) -> tuple[TDOAResult, ...]:
        """Return candidates, quality metrics, bounds, and explicit validity."""
        ...


class CoarseEnvelopeEstimator:
    """Extension point for envelope/onset timing before carrier-cycle refinement."""

    def estimate(
        self, detection: PingDetection, config: DSPConfig,  # noqa: ARG002 - unfinished estimator input
    ) -> tuple[TDOAResult, ...]:
        """Report unfinished coarse timing rather than silently generating zero delays."""
        return pending_pairs(config, 'TODO: coarse envelope/onset estimator')


class FineWaveformEstimator:
    """
    Extension point for correlation/phase candidates spaced by roughly 1/f0.

    TODO: combine coarse onset intervals with correlation or carrier phase, retain
    competing cycle peaks, and reject weak/coherence-poor/multipath measurements.
    Never assume the largest waveform-correlation peak is the physical delay.
    """

    def estimate(
        self, detection: PingDetection, config: DSPConfig,  # noqa: ARG002 - unfinished estimator input
    ) -> tuple[TDOAResult, ...]:
        """Return explicit not_implemented statuses until calibrated timing is available."""
        return pending_pairs(config, 'TODO: coarse/fine tone-burst delay candidate estimation')


def pending_pairs(config: DSPConfig, reason: str) -> tuple[TDOAResult, ...]:
    """Create explicit unfinished results for all six pairs."""
    geometry = config.geometry
    return tuple(TDOAResult(
        (geometry.channel_ids[i], geometry.channel_ids[j]), (),
        float(np.linalg.norm(geometry.coordinates_m[i] - geometry.coordinates_m[j]) / config.sound_speed_m_s),
        Status.NOT_IMPLEMENTED, reason,
    ) for i, j in PAIRS)


def calibrate_delays(results: tuple[TDOAResult, ...], config: DSPConfig) -> tuple[TDOAResult, ...]:
    """Subtract offset_i - offset_j from raw delay candidates without moving samples."""
    calibrated = []
    for result in results:
        i, j = (config.geometry.channel_ids.index(channel) for channel in result.channel_pair)
        offset = config.geometry.timing_offsets_s[i] - config.geometry.timing_offsets_s[j]
        calibrated.append(replace(result, candidates=tuple(
            replace(candidate, delay_s=float(candidate.delay_s - offset)) for candidate in result.candidates
        )))
    return tuple(calibrated)


def enforce_physical_bounds(results: tuple[TDOAResult, ...], config: DSPConfig) -> tuple[TDOAResult, ...]:
    """Recompute physical bounds from geometry and discard impossible/nonfinite candidates."""
    expected = {(config.geometry.channel_ids[i], config.geometry.channel_ids[j]) for i, j in PAIRS}
    if len(results) != len(PAIRS) or {result.channel_pair for result in results} != expected:
        msg = 'estimator must report exactly the six configured i<j pairs'
        raise ValueError(msg)
    bounded = []
    for result in results:
        i, j = (config.geometry.channel_ids.index(channel) for channel in result.channel_pair)
        bound = float(np.linalg.norm(config.geometry.coordinates_m[i] - config.geometry.coordinates_m[j])
                      / config.sound_speed_m_s)
        candidates = tuple(candidate for candidate in result.candidates
                           if np.isfinite(candidate.delay_s) and abs(candidate.delay_s) <= bound)
        status, reason = result.status, result.reason
        if status == Status.OK and not candidates:
            status, reason = Status.INVALID, 'no candidates within physical propagation bound'
        bounded.append(TDOAResult(result.channel_pair, candidates, bound, status, reason))
    return tuple(bounded)
