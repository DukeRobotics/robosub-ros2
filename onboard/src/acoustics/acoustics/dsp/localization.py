"""Reference far-field candidate fitting; no single-ping range claim."""
from itertools import product
from math import atan2, prod
from typing import Protocol

import numpy as np

from acoustics.dsp.config import DSPConfig
from acoustics.dsp.contracts import DirectionCandidate, LocalizationResult, Status, TDOAResult
from acoustics.dsp.tdoa import enforce_physical_bounds

PLANAR_RANK = 2
MIN_DIRECTION_NORM = 1e-12


class NearFieldEstimator(Protocol):
    """Separate extension point for spherical propagation and position hypotheses."""

    def estimate(
        self, results: tuple[TDOAResult, ...], timestamp_s: float, config: DSPConfig,
    ) -> LocalizationResult:
        """Return position hypotheses with explicit uncertainty and ambiguity."""
        ...


class PendingNearFieldEstimator:
    """Unimplemented position estimator; four channels do not guarantee usable range."""

    def estimate(
        self, results: tuple[TDOAResult, ...], timestamp_s: float, config: DSPConfig,  # noqa: ARG002 - extension placeholder
    ) -> LocalizationResult:
        """Report the absence of a near-field algorithm explicitly."""
        return LocalizationResult(timestamp_s, config.geometry.frame_id, Status.NOT_IMPLEMENTED,
                                  reason='TODO: near-field position estimator and observability checks')


class FarFieldLocalizer:
    """
    Fit c*tau_ij = -(h_i-h_j) dot u; u points toward the source.

    Reference least-squares candidate fit, not a noise-weighted production estimator.
    Three latent independent delays fit six pairs; cycle consistency and geometric
    residuals reject contradictory measurements. No uncertainty model is implemented.
    """

    def localize(
        self, results: tuple[TDOAResult, ...], timestamp_s: float, config: DSPConfig,
    ) -> LocalizationResult:
        """Enumerate bounded delay combinations and preserve plausible direction alternatives."""
        geometry = config.geometry
        results = enforce_physical_bounds(results, config)
        if any(result.status == Status.NOT_IMPLEMENTED for result in results):
            return LocalizationResult(timestamp_s, geometry.frame_id, Status.NOT_IMPLEMENTED,
                                      reason='delay estimation is not implemented')
        if any(result.status != Status.OK or not result.candidates for result in results):
            return LocalizationResult(timestamp_s, geometry.frame_id, Status.INVALID,
                                      reason='all six pair results must contain valid delay candidates')
        count = prod(len(result.candidates) for result in results)
        if count > config.max_candidate_combinations:
            return LocalizationResult(timestamp_s, geometry.frame_id, Status.AMBIGUOUS,
                                      ambiguity='candidate_limit',
                                      reason='candidate search limit exceeded; no truncation')
        indices = [(geometry.channel_ids.index(result.channel_pair[0]),
                    geometry.channel_ids.index(result.channel_pair[1])) for result in results]
        baselines = np.asarray([geometry.coordinates_m[i] - geometry.coordinates_m[j] for i, j in indices])
        incidence = np.zeros((len(results), 4))
        for row, (i, j) in enumerate(indices):
            incidence[row, i], incidence[row, j] = 1, -1
        rank = np.linalg.matrix_rank(baselines)
        if rank < PLANAR_RANK:
            return LocalizationResult(timestamp_s, geometry.frame_id, Status.INVALID,
                                      reason='array is collinear; direction is underdetermined')
        _, _, vh = np.linalg.svd(baselines)
        normal = vh[-1]
        # Fix the otherwise arbitrary SVD normal sign for a reproducible planar constraint.
        if normal[np.argmax(np.abs(normal))] < 0:
            normal = -normal
        directions = []
        for candidates in product(*(result.candidates for result in results)):
            delays = np.asarray([candidate.delay_s for candidate in candidates])
            arrivals = np.linalg.lstsq(incidence, delays, rcond=None)[0]
            if np.max(np.abs(incidence @ arrivals - delays)) > config.pair_consistency_tolerance_s:
                continue
            estimate = np.linalg.lstsq(baselines, -config.sound_speed_m_s * delays, rcond=None)[0]
            length = np.linalg.norm(estimate)
            if rank == PLANAR_RANK:
                if length > 1 + 1e-9:
                    continue
                height = np.sqrt(max(0.0, 1 - length ** 2))
                vectors = [estimate + height * normal, estimate - height * normal]
                if config.planar_constraint == 'positive_normal':
                    vectors = vectors[:1]
                elif config.planar_constraint == 'negative_normal':
                    vectors = vectors[1:]
            else:
                if length < MIN_DIRECTION_NORM:
                    continue
                vectors = [estimate / length]
            for vector in vectors:
                residual = float(np.sqrt(np.mean((baselines @ vector + config.sound_speed_m_s * delays) ** 2)))
                if residual > config.fit_tolerance_m:
                    continue
                if any(np.allclose(vector, direction.unit_vector, atol=1e-6, rtol=0) for direction in directions):
                    continue
                directions.append(DirectionCandidate(
                    tuple(float(value) for value in vector),
                    atan2(float(vector[1]), float(vector[0])),
                    atan2(float(vector[2]), float(np.hypot(vector[0], vector[1]))), residual,
                ))
        if not directions:
            return LocalizationResult(timestamp_s, geometry.frame_id, Status.INVALID,
                                      reason='inconsistent pairs or no unit direction within fit tolerance')
        ambiguous = len(directions) > 1
        ambiguity = 'planar_mirror_or_delay_candidates' if rank == PLANAR_RANK else 'delay_candidates'
        return LocalizationResult(
            timestamp_s, geometry.frame_id, Status.AMBIGUOUS if ambiguous else Status.OK,
            tuple(directions), ambiguity=ambiguity if ambiguous else 'none',
        )
