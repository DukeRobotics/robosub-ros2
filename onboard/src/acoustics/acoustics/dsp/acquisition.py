"""Replay native Logic2 exports using the copied upstream hydrophone loader."""
import contextlib
import io
import json
from pathlib import Path

import numpy as np

from acoustics.dsp.contracts import CHANNEL_COUNT, SampleBlock
from acoustics.upstream_adapter import CalibratedHydrophoneArray


def load_saleae_block(path: Path, channel_ids: tuple[str, ...] = ('A0', 'A1', 'A2', 'A3')) -> SampleBlock:
    """Load one native capture and validate complete synchronized four-channel data."""
    if not path.is_dir():
        msg = f'Expected a Logic2 capture directory: {path}'
        raise ValueError(msg)
    metadata_path = path / 'capture.json'
    metadata = {}
    if metadata_path.is_file():
        with metadata_path.open(encoding='utf-8') as stream:
            metadata = json.load(stream)
        if tuple(metadata['channel_ids']) != channel_ids:
            msg = 'recorded Saleae channel mapping does not match configured channel IDs'
            raise ValueError(msg)
    csv_files = list(path.glob('*.csv'))
    if len(csv_files) > 1:
        msg = 'multiple CSV exports in one capture directory are ambiguous'
        raise ValueError(msg)
    if not csv_files:
        for channel in range(CHANNEL_COUNT):
            if len(list(path.glob(f'*_{channel}.bin'))) != 1:
                msg = f'expected exactly one Saleae binary export for analog channel {channel}'
                raise ValueError(msg)
    array = CalibratedHydrophoneArray(
        np.zeros(CHANNEL_COUNT), timestamp_origin_s=metadata.get('timestamp_origin_s', 0.0),
    )
    with contextlib.redirect_stdout(io.StringIO()):
        array.load_from_path(str(path))
    return array.sample_block(channel_ids, metadata.get('timestamp_reference', 'recording_relative'))
