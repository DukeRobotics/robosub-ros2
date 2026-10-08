"""Packaging, synchronization checks, and timing-calibration hooks around copied upstream code."""
import contextlib
import io
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from acoustics.dsp.config import DSPConfig
from acoustics.dsp.contracts import CHANNEL_COUNT, HydrophoneAnalysis, PingDetection, SampleBlock, Status
from acoustics.dsp.detection import extract_window
from acoustics.upstream.analyzers import NearbyAnalyzer, TOAEnvelopeAnalyzer
from acoustics.upstream.hydrophones.hydrophone import Hydrophone
from acoustics.upstream.hydrophones.hydrophone_array import HydrophoneArray

MODEL_NAME = 'proximity_classifier_10ft_threshold_2026-04-12--23-04-00.pkl'


class CalibratedHydrophoneArray(HydrophoneArray):
    """
    Keep upstream loading/DC removal and subtract configured electronic timing offsets.

    Positive offset means a channel records late. Its calibrated TOA is raw TOA minus
    offset. Samples and original sample indices are never independently shifted.
    """

    def __init__(
        self, timing_offsets_s: NDArray[np.float64], sample_rate_hz: float = 781250.0,
        timestamp_origin_s: float = 0.0,
    ) -> None:
        super().__init__(sampling_freq=sample_rate_hz, selected=[True] * CHANNEL_COUNT)
        self.timing_offsets_s = timing_offsets_s
        self.timestamp_origin_s = timestamp_origin_s
        self.raw_samples: dict[int, NDArray[np.float64]] = {}
        self.raw_times: dict[int, NDArray[np.float64]] = {}

    def _update_hydrophone(
        self, hydro: Hydrophone, times: NDArray[np.float64], signal: NDArray[np.float64],
    ) -> None:
        """Run the original upstream update and add only the timing calibration hook."""
        index = self.hydrophones.index(hydro)
        self.raw_samples[index] = np.asarray(signal, dtype=float)
        self.raw_times[index] = np.asarray(times, dtype=float)
        super()._update_hydrophone(hydro, times, signal)
        hydro.times = times + self.timestamp_origin_s - self.timing_offsets_s[index]

    def sample_block(self, channel_ids: tuple[str, ...], timestamp_reference: str) -> SampleBlock:
        """Reject partial/misaligned exports and retain the original four-channel grid."""
        if set(self.raw_samples) != set(range(CHANNEL_COUNT)):
            msg = 'Saleae capture must load all four channels 0-3'
            raise ValueError(msg)
        reference_times = self.raw_times[0]
        period = self.hydrophones[0].sampling_period
        if period is None or not np.isfinite(period) or period <= 0:
            msg = 'invalid Saleae sample period'
            raise ValueError(msg)
        for index, hydro in enumerate(self.hydrophones):
            times = self.raw_times[index]
            if (len(times) != len(reference_times) or len(times) == 0
                    or not np.isclose(hydro.sampling_period, period, rtol=1e-12, atol=0)
                    or not np.allclose(times, reference_times, rtol=0, atol=max(1e-12, period * 1e-6))
                    or not np.allclose(np.diff(times), period, rtol=1e-8, atol=1e-12)):
                msg = 'Saleae channels have different lengths, rates, or original acquisition timelines'
                raise ValueError(msg)
        return SampleBlock(
            samples=np.column_stack([self.raw_samples[index] for index in range(CHANNEL_COUNT)]),
            channel_ids=channel_ids, sample_rate_hz=1 / period,
            timestamp_s=float(reference_times[0] + self.timestamp_origin_s), units='V',
            timestamp_reference=timestamp_reference,
        )


def array_from_block(block: SampleBlock, config: DSPConfig) -> CalibratedHydrophoneArray:
    """Use the copied upstream hydrophone update for synchronized in-memory recordings."""
    if set(block.channel_ids) != set(config.geometry.channel_ids):
        msg = 'recording channel IDs do not match geometry'
        raise ValueError(msg)
    if not np.isclose(block.sample_rate_hz, config.sample_rate_hz, rtol=1e-9):
        msg = 'recording and configured sample rates differ; implicit resampling is forbidden'
        raise ValueError(msg)
    array = CalibratedHydrophoneArray(config.geometry.timing_offsets_s, block.sample_rate_hz)
    times = block.timestamp_s + np.arange(len(block.samples)) / block.sample_rate_hz
    for index, channel_id in enumerate(config.geometry.channel_ids):
        hydro = array.hydrophones[index]
        hydro.sampling_period = 1 / block.sample_rate_hz
        array._update_hydrophone(  # noqa: SLF001 - explicit adapter hook into the copied upstream container
            hydro, times, block.samples[:, block.channel_ids.index(channel_id)],
        )
    return array


class UpstreamEnvelopeDetector:
    """
    Adapt the copied TOA/nearby analyzers into one shared four-channel DSP window.

    Upstream detects one arrival per channel per capture, including its documented
    envelope-peak fallback. Multiple-burst detection and robust TDOA remain unfinished.
    """

    def __init__(self) -> None:
        self.channel_analysis: tuple[HydrophoneAnalysis, ...] = ()
        self.nearby_analyzer: NearbyAnalyzer | None = None
        self.nearby_key: tuple | None = None

    def detect(self, block: SampleBlock, config: DSPConfig) -> tuple[PingDetection, ...]:
        """Apply unchanged upstream analysis with channel timing offsets and common cropping."""
        self.channel_analysis = ()
        array = array_from_block(block, config)
        band = {
            'search_band_min': config.pinger_frequency_hz - config.filter_bandwidth_hz / 2,
            'search_band_max': config.pinger_frequency_hz + config.filter_bandwidth_hz / 2,
            'filter_order': config.filter_order, 'plot_results_flag': False,
        }
        analyzer = TOAEnvelopeAnalyzer(
            threshold_sigma=config.toa_threshold_sigma, raw_signal_threshold=config.raw_signal_threshold,
            margin_front=config.margin_front_s, margin_end=config.margin_end_s, **band,
        )
        toa = analyzer.analyze_array(array)['results']
        nearby = None
        if config.enable_nearby:
            model_path = config.nearby_model_path or str(Path(__file__).parent / 'upstream' / 'artifacts' / MODEL_NAME)
            key = (model_path, *band.values())
            if self.nearby_key != key:
                with contextlib.redirect_stdout(io.StringIO()):
                    self.nearby_analyzer = NearbyAnalyzer(model_path=model_path, **band)
                self.nearby_key = key
            nearby = self.nearby_analyzer.analyze_array(array)['results']
        self.channel_analysis = tuple(HydrophoneAnalysis(
            channel_id=config.geometry.channel_ids[index],
            toa_sample_index=block.first_sample_index + int(result['toa_idx']),
            raw_toa_s=block.timestamp_s + int(result['toa_idx']) / block.sample_rate_hz,
            calibrated_toa_s=float(result['toa_time']), valid=bool(result['is_valid']),
            validation_reason=result['validity_reason'],
            nearby=nearby[index]['is_nearby'] if nearby is not None else None,
            nearby_confidence=nearby[index]['confidence'] if nearby is not None else None,
        ) for index, result in enumerate(toa))
        if all(result.validation_reason == 'WEAK_SIGNAL' for result in self.channel_analysis):
            return ()
        indices = [int(result['toa_idx']) for result in toa]
        onset = min(indices)
        pre = int(np.ceil(config.pre_trigger_s * block.sample_rate_hz))
        post = int(np.ceil((config.window_duration_s + config.post_trigger_s) * block.sample_rate_hz))
        start, stop = max(0, onset - pre), min(len(block.samples), max(indices) + post)
        filtered = SampleBlock(
            samples=np.column_stack([result['filtered_signal'] for result in toa]),
            channel_ids=config.geometry.channel_ids, sample_rate_hz=block.sample_rate_hz,
            timestamp_s=block.timestamp_s, units=block.units, first_sample_index=block.first_sample_index,
            timestamp_reference=block.timestamp_reference,
        )
        window = extract_window(filtered, start, stop)
        complete = start == onset - pre and stop == max(indices) + post
        valid = all(result.valid for result in self.channel_analysis) and complete
        spectrum = np.abs(np.fft.rfft(window.samples, axis=0)).sum(axis=1)
        frequencies = np.fft.rfftfreq(len(window.samples), d=1 / block.sample_rate_hz)
        reason = '; '.join(f'{result.channel_id}: {result.validation_reason}' for result in self.channel_analysis
                           if not result.valid)
        if not complete:
            reason = f'{reason}; truncated common window'.lstrip('; ')
        return (PingDetection(
            window=window, start_sample_index=window.first_sample_index,
            end_sample_index=block.first_sample_index + stop,
            onset_sample_index=block.first_sample_index + onset,
            frequency_hz=float(frequencies[np.argmax(spectrum)]),
            quality={'complete_window': float(complete), 'upstream_toa_threshold_sigma': config.toa_threshold_sigma},
            status=Status.OK if valid else Status.INVALID, reason=reason,
        ),)
