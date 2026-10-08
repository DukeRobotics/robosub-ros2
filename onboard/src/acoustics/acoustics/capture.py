"""One-request capture workflow using the copied upstream Logic2 adapter and DSP core."""
import json
import time
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path
from typing import Protocol
from uuid import uuid4

from acoustics.dsp.acquisition import load_saleae_block
from acoustics.dsp.config import DSPConfig
from acoustics.dsp.contracts import ProcessingResult
from acoustics.dsp.pipeline import Pipeline


class LogicInterface(Protocol):
    """Upstream Logic2 connection/capture interface, injectable for hardware-free tests."""

    def open(self) -> None:
        """Connect to the already-running Logic2 automation server."""
        ...

    def capture(
        self, seconds: float, prefix: str, base_dir: str, sample_rate: int, formats: list[str],
    ) -> tuple[dict[str, str], str]:
        """Capture analog channels 0-3 and export completed raw data."""
        ...

    def close(self) -> None:
        """Close the automation connection."""
        ...


class CaptureProcessor:
    """Capture exactly once per call, replay that native export, and return its analysis."""

    def __init__(
        self, config: DSPConfig, logic_factory: Callable[[], LogicInterface] | None = None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.config = config
        self.pipeline = Pipeline(config)
        self.logic_factory = logic_factory
        self.clock = clock

    def run(self) -> tuple[ProcessingResult, Path]:
        """Execute upstream open -> timed capture/export -> analysis -> close for one request."""
        if self.logic_factory is None:
            # Importing the hardware SDK is unnecessary for offline processing.
            from acoustics.upstream.logic.logic2 import Logic2  # noqa: PLC0415 - live acquisition only

            logic = Logic2(is_mock=self.config.use_mock_device)
        else:
            logic = self.logic_factory()
        logic.open()
        try:
            prefix = f'{datetime.now(UTC):%Y-%m-%d--%H-%M-%S}_{uuid4().hex[:8]}'
            # This is a host-side marker before capture(), not a hardware timestamp.
            origin = self.clock()
            _, output_dir = logic.capture(
                seconds=self.config.capture_duration_s, prefix=prefix, base_dir=self.config.capture_output_dir,
                sample_rate=int(self.config.sample_rate_hz), formats=['bin'],
            )
            path = Path(output_dir)
            metadata = {
                'timestamp_origin_s': origin, 'timestamp_reference': 'host_before_capture_call',
                'channel_ids': list(self.config.geometry.channel_ids), 'analog_channels': [0, 1, 2, 3],
                'sample_rate_hz': self.config.sample_rate_hz,
                'timing_offsets_s': self.config.geometry.timing_offsets_s.tolist(),
            }
            (path / 'capture.json').write_text(json.dumps(metadata, allow_nan=False), encoding='utf-8')
            block = load_saleae_block(path, self.config.geometry.channel_ids)
            return self.pipeline.process(block), path
        finally:
            logic.close()
