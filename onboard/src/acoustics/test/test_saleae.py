"""Capture/export, copied-processing equivalence, native replay, and timing-calibration checks."""
# ruff: noqa: PLR2004 - synthetic channel/timing fixtures use explicit expected values
import contextlib
import io
import json
import struct
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from acoustics.capture import CaptureProcessor
from acoustics.dsp.acquisition import load_saleae_block
from acoustics.dsp.config import DSPConfig, load_config
from acoustics.dsp.contracts import PAIRS, DelayCandidate, Status, TDOAResult
from acoustics.dsp.pipeline import Pipeline
from acoustics.dsp.tdoa import calibrate_delays, enforce_physical_bounds
from acoustics.replay import main as replay_main
from acoustics.upstream.analyzers import NearbyAnalyzer, TOAEnvelopeAnalyzer
from acoustics.upstream.hydrophones.hydrophone_array import HydrophoneArray
from acoustics.upstream.logic import logic2
from acoustics.upstream_adapter import MODEL_NAME, CalibratedHydrophoneArray, array_from_block

CONFIG_PATH = Path(__file__).parents[1] / 'config' / 'dsp.yaml'


@pytest.fixture
def config() -> DSPConfig:
    """Use upstream hardware defaults with short-recording validation margins for fixtures."""
    return replace(load_config(CONFIG_PATH), enable_nearby=False, margin_front_s=0.0, margin_end_s=0.0,
                   raw_signal_threshold=0.1, window_duration_s=0.0005)


def write_capture(path: Path, config: DSPConfig, start_s: float = 0.00007512175) -> Path:
    """Create actual Saleae v0 analog exports with DC bias and known channel delays."""
    path.mkdir(parents=True, exist_ok=True)
    signal = np.zeros(12000)
    grid = np.arange(800)
    signal[4000:4800] = 0.8 * np.sin(2 * np.pi * config.pinger_frequency_hz * grid / config.sample_rate_hz)
    for index in range(4):
        samples = np.asarray(5.0 + np.roll(signal, index * 3), dtype='<f4')
        header = struct.pack('<8siidQQQ', b'<SALEAE>', 0, 1, start_s, int(config.sample_rate_hz), 1, len(samples))
        (path / f'analog_{index}.bin').write_bytes(header + samples.tobytes())
    return path


def test_native_replay_matches_copied_upstream_loading(tmp_path: Path, config: DSPConfig) -> None:
    """Native replay retains raw voltage values, and processing matches upstream DC removal/TOA."""
    path = write_capture(tmp_path / 'capture', config)
    original = HydrophoneArray(sampling_freq=config.sample_rate_hz)
    with contextlib.redirect_stdout(io.StringIO()):
        original.load_from_path(str(path))
    block = load_saleae_block(path)
    assert block.channel_ids == ('A0', 'A1', 'A2', 'A3')
    assert block.units == 'V'
    assert block.timestamp_reference == 'recording_relative'
    assert block.timestamp_s == pytest.approx(0.00007512175)
    assert block.sample_rate_hz == config.sample_rate_hz
    assert block.samples.mean() == pytest.approx(5.0, abs=0.001)
    configured = array_from_block(block, config)
    for baseline, calibrated in zip(original.hydrophones, configured.hydrophones, strict=True):
        # Upstream binary loading subtracts the float32 mean. In-memory adapter uses the
        # same values in float64, so allow only its floating-point rounding difference.
        np.testing.assert_allclose(baseline.signal, calibrated.signal, atol=1e-6)
        np.testing.assert_allclose(baseline.times, calibrated.times, atol=1e-12)
    analyzer = TOAEnvelopeAnalyzer(threshold_sigma=config.toa_threshold_sigma,
                                  raw_signal_threshold=config.raw_signal_threshold,
                                  margin_front=0.0, margin_end=0.0, filter_order=config.filter_order,
                                  search_band_min=30000, search_band_max=34000, plot_results_flag=False)
    baseline = analyzer.analyze_array(configured)['results']
    result = Pipeline(config).process(block)
    assert result.status == Status.NOT_IMPLEMENTED
    assert [item.toa_sample_index for item in result.channel_analysis] == [int(item['toa_idx']) for item in baseline]
    assert [item.calibrated_toa_s for item in result.channel_analysis] == pytest.approx(
        [item['toa_time'] for item in baseline],
    )
    assert result.detections[0].window.timestamp_reference == 'recording_relative'


def test_bundled_nearby_classifier_matches_copied_analyzer(tmp_path: Path, config: DSPConfig) -> None:
    """The default model and all-four-channel classifier results match direct upstream processing."""
    block = load_saleae_block(write_capture(tmp_path / 'capture', config))
    configured = replace(config, enable_nearby=True)
    model_path = CONFIG_PATH.parent.parent / 'acoustics' / 'upstream' / 'artifacts' / MODEL_NAME
    with contextlib.redirect_stdout(io.StringIO()):
        classifier = NearbyAnalyzer(model_path=str(model_path), search_band_min=30000,
                                    search_band_max=34000, filter_order=config.filter_order,
                                    plot_results_flag=False)
    expected = classifier.analyze_array(array_from_block(block, configured))['results']
    result = Pipeline(configured).process(block)
    assert len(result.channel_analysis) == 4
    assert [item.nearby for item in result.channel_analysis] == [item['is_nearby'] for item in expected]
    assert [item.nearby_confidence for item in result.channel_analysis] == pytest.approx(
        [item['confidence'] for item in expected],
    )
    assert result.status == Status.NOT_IMPLEMENTED


def test_native_replay_cli(
    tmp_path: Path, config: DSPConfig, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture,
) -> None:
    """The public CLI processes a native export and reports invalid input without stopping replay."""
    path = write_capture(tmp_path / 'capture', config)
    values = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    values.update(enable_nearby=False, margin_front_s=0.0, margin_end_s=0.0,
                  raw_signal_threshold=0.1, window_duration_s=0.0005)
    replay_config = tmp_path / 'test.yaml'
    replay_config.write_text(yaml.safe_dump(values), encoding='utf-8')
    monkeypatch.setattr('sys.argv', ['acoustics_replay', '--config', str(replay_config),
                                   str(path), str(tmp_path / 'missing')])
    replay_main()
    outputs = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert outputs[0]['result']['status'] == 'not_implemented'
    assert outputs[0]['result']['detections'][0]['window']['channel_ids'] == ['A0', 'A1', 'A2', 'A3']
    assert outputs[1]['result']['status'] == 'invalid'
    assert 'capture directory' in outputs[1]['result']['reason']


def test_offset_calibration_changes_times_not_waveforms(tmp_path: Path, config: DSPConfig) -> None:
    """Calibrating a late channel subtracts its offset and keeps common waveform indices intact."""
    path = write_capture(tmp_path / 'capture', config)
    block = load_saleae_block(path)
    offsets = np.array([0, 3, 6, 9]) / config.sample_rate_hz
    configured = replace(config, geometry=replace(config.geometry, timing_offsets_s=offsets))
    before = Pipeline(config).process(block)
    after = Pipeline(configured).process(block)
    for index, (raw, corrected) in enumerate(zip(before.channel_analysis, after.channel_analysis, strict=True)):
        assert raw.toa_sample_index == corrected.toa_sample_index
        assert raw.raw_toa_s == corrected.raw_toa_s
        assert corrected.calibrated_toa_s == pytest.approx(raw.calibrated_toa_s - offsets[index], abs=1e-12)
    np.testing.assert_array_equal(before.detections[0].window.samples, after.detections[0].window.samples)
    assert before.detections[0].start_sample_index == after.detections[0].start_sample_index
    assert before.detections[0].end_sample_index == after.detections[0].end_sample_index
    assert before.detections[0].window.timestamp_s == after.detections[0].window.timestamp_s
    # The calibration subclass extends upstream loading itself, without changing its samples.
    array = CalibratedHydrophoneArray(offsets, config.sample_rate_hz)
    with contextlib.redirect_stdout(io.StringIO()):
        array.load_from_path(str(path))
    assert array.hydrophones[1].times[0] == pytest.approx(block.timestamp_s - offsets[1])


def test_delay_calibration_sign_precedes_physical_bounds(config: DSPConfig) -> None:
    """Electronic delays are corrected exactly once before rejecting impossible propagation delays."""
    offsets = np.array([0.0001, 0.0, 0.0, 0.0])
    configured = replace(config, geometry=replace(config.geometry, timing_offsets_s=offsets))
    results = tuple(TDOAResult(
        (configured.geometry.channel_ids[i], configured.geometry.channel_ids[j]),
        (DelayCandidate(float(offsets[i] - offsets[j])),), 1.0, Status.OK,
    ) for i, j in PAIRS)
    assert enforce_physical_bounds(results, configured)[0].status == Status.INVALID
    corrected = enforce_physical_bounds(calibrate_delays(results, configured), configured)
    assert all(result.status == Status.OK for result in corrected)
    assert all(result.candidates[0].delay_s == 0.0 for result in corrected)


@pytest.mark.parametrize('problem', ['missing', 'start', 'rate', 'truncated', 'duplicate'])
def test_native_replay_rejects_bad_synchronization(tmp_path: Path, config: DSPConfig, problem: str) -> None:
    """Incomplete or differently clocked channel exports cannot silently become a valid block."""
    path = write_capture(tmp_path / 'capture', config)
    channel = path / 'analog_3.bin'
    if problem == 'missing':
        channel.unlink()
    elif problem == 'duplicate':
        (path / 'other_3.bin').write_bytes(channel.read_bytes())
    else:
        data = bytearray(channel.read_bytes())
        if problem == 'start':
            struct.pack_into('<d', data, 16, 0.001)
        elif problem == 'rate':
            struct.pack_into('<Q', data, 24, int(config.sample_rate_hz / 2))
        else:
            data = data[:-4]
        channel.write_bytes(data)
    with pytest.raises(ValueError, match='Saleae'):
        load_saleae_block(path)


def test_service_capture_calls_upstream_logic2_once(
    tmp_path: Path, config: DSPConfig, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Exercise the copied SDK flow: connect, one four-channel capture, wait/export, close."""
    events = []
    sdk = {}

    class FakeCapture:
        def wait(self) -> None:
            """Record completion before export."""
            events.append('wait')

        def export_raw_data_binary(self, directory: str) -> None:
            """Export a real Saleae-format recording for the remaining pipeline."""
            events.append('export')
            write_capture(Path(directory), config)

        def close(self) -> None:
            """Record release of this capture."""
            events.append('capture_close')

    class FakeManager:
        def get_devices(self, include_simulation_devices: bool) -> list:
            """Expose a physical Logic8."""
            assert not include_simulation_devices
            return [SimpleNamespace(device_id='logic8', device_type=3)]

        def start_capture(self, **kwargs: object) -> FakeCapture:
            """Record the actual SDK configurations built by copied upstream code."""
            events.append('capture')
            sdk.update(kwargs)
            return FakeCapture()

        def close(self) -> None:
            """Record automation connection cleanup."""
            events.append('manager_close')

    def connect() -> FakeManager:
        """Stand in for Manager.connect without launching software or contacting hardware."""
        events.append('connect')
        return FakeManager()

    monkeypatch.setattr(logic2.Manager, 'connect', connect)
    processor = CaptureProcessor(replace(config, capture_output_dir=str(tmp_path)), clock=lambda: 123.0)
    assert not events  # Construction must not capture or connect.
    result, directory = processor.run()
    assert events == ['connect', 'capture', 'wait', 'export', 'capture_close', 'manager_close']
    assert sdk['device_id'] == 'logic8'
    assert sdk['device_configuration'].enabled_analog_channels == [0, 1, 2, 3]
    assert sdk['device_configuration'].analog_sample_rate == 781250
    assert sdk['capture_configuration'].capture_mode.duration_seconds == config.capture_duration_s
    assert result.status == Status.NOT_IMPLEMENTED
    replay = load_saleae_block(directory)
    assert replay.timestamp_reference == 'host_before_capture_call'
    assert replay.timestamp_s == pytest.approx(123.00007512175)
    # A second request reconnects and captures once again, with a distinct retained export.
    _, second_directory = processor.run()
    assert second_directory != directory
    assert events == ['connect', 'capture', 'wait', 'export', 'capture_close', 'manager_close'] * 2


def test_capture_failure_closes_connection(tmp_path: Path, config: DSPConfig) -> None:
    """Service capture failures release the automation connection and propagate an explicit error."""
    events = []

    class FailedLogic:
        def open(self) -> None:
            """Record successful connection."""
            events.append('open')

        def capture(self, **kwargs: object) -> None:  # noqa: ARG002 - SDK-shaped failure stub
            """Simulate an SDK capture failure."""
            msg = 'capture failed'
            raise RuntimeError(msg)

        def close(self) -> None:
            """Record guaranteed cleanup."""
            events.append('close')

    processor = CaptureProcessor(replace(config, capture_output_dir=str(tmp_path)), logic_factory=FailedLogic)
    with pytest.raises(RuntimeError, match='capture failed'):
        processor.run()
    assert events == ['open', 'close']
