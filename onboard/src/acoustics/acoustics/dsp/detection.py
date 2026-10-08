"""Burst-detection interface and timing-preserving shared-window extraction."""
from typing import Protocol

from acoustics.dsp.config import DSPConfig
from acoustics.dsp.contracts import PingDetection, SampleBlock


class BurstDetector(Protocol):
    """Extension point for detection on raw synchronized four-channel recordings."""

    def detect(self, block: SampleBlock, config: DSPConfig) -> tuple[PingDetection, ...]:
        """Return common filtered windows without shifting individual channels."""
        ...


def extract_window(block: SampleBlock, start: int, stop: int) -> SampleBlock:
    """Slice all four channels together, retaining acquisition time and original indices."""
    if not 0 <= start < stop <= len(block.samples):
        msg = 'window bounds must lie within the sample block'
        raise ValueError(msg)
    return SampleBlock(
        block.samples[start:stop].copy(), block.channel_ids, block.sample_rate_hz,
        block.timestamp_s + start / block.sample_rate_hz, block.units, block.first_sample_index + start,
        timestamp_reference=block.timestamp_reference,
    )
