"""Common orchestration for offline replay and ROS, with injectable algorithms."""
from dataclasses import replace

import numpy as np

from acoustics.dsp.config import DSPConfig
from acoustics.dsp.contracts import LocalizationResult, ProcessingResult, SampleBlock, Status
from acoustics.dsp.detection import BurstDetector
from acoustics.dsp.localization import FarFieldLocalizer
from acoustics.dsp.tdoa import (
    DelayEstimator,
    FineWaveformEstimator,
    calibrate_delays,
    enforce_physical_bounds,
    pending_pairs,
)
from acoustics.upstream_adapter import UpstreamEnvelopeDetector


class Pipeline:
    """Process complete independent blocks; streaming overlap/state is future work."""

    def __init__(
        self, config: DSPConfig, detector: BurstDetector | None = None, estimator: DelayEstimator | None = None,
    ) -> None:
        self.config = config
        self.detector = detector if detector is not None else UpstreamEnvelopeDetector()
        self.estimator = estimator if estimator is not None else FineWaveformEstimator()
        self.localizer = FarFieldLocalizer()

    def process(self, block: SampleBlock) -> ProcessingResult:
        """Run upstream detection on raw samples, estimate six delays, and localize each burst."""
        if self.config.clipping_enabled and np.any(np.abs(block.samples) >= self.config.clipping_level):
            return ProcessingResult(Status.INVALID, reason='raw block reaches configured clipping level')
        detections = self.detector.detect(block, self.config)
        channel_analysis = getattr(self.detector, 'channel_analysis', ())
        if not detections:
            return ProcessingResult(Status.NO_DETECTION, reason='no usable upstream burst detection',
                                    channel_analysis=channel_analysis)
        delays, localizations = [], []
        for detection in detections:
            # Use the trigger timestamp rather than the beginning of the pre-trigger window.
            timestamp = detection.window.timestamp_s + (
                detection.onset_sample_index - detection.start_sample_index
            ) / block.sample_rate_hz
            if detection.status != Status.OK:
                delays.append(tuple(replace(pair, status=Status.INVALID, reason=detection.reason)
                                    for pair in pending_pairs(self.config, detection.reason)))
                localizations.append(LocalizationResult(
                    timestamp, self.config.geometry.frame_id,
                    Status.INVALID, reason=detection.reason,
                ))
                continue
            pairs = self.estimator.estimate(detection, self.config)
            pairs = calibrate_delays(pairs, self.config)
            pairs = enforce_physical_bounds(pairs, self.config)
            delays.append(pairs)
            localizations.append(self.localizer.localize(pairs, timestamp, self.config))
        statuses = {localization.status for localization in localizations}
        status = next((item for item in (Status.NOT_IMPLEMENTED, Status.AMBIGUOUS, Status.INVALID)
                       if item in statuses), Status.OK)
        return ProcessingResult(status, detections, tuple(delays), tuple(localizations),
                                channel_analysis=channel_analysis)
