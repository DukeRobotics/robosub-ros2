"""Run copied upstream hydrophone processing on native Logic2 exports without ROS/hardware."""
import argparse
import json
from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import Any

import numpy as np

from acoustics.dsp.acquisition import load_saleae_block
from acoustics.dsp.config import load_config
from acoustics.dsp.pipeline import Pipeline


def json_value(value: Any) -> Any:  # noqa: ANN401 - recursive serialization of contract types
    """Serialize contracts to JSON; omit large waveform arrays in replay summaries."""
    if is_dataclass(value):
        return {field.name: json_value(getattr(value, field.name)) for field in fields(value)
                if field.name != 'samples'}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    return value


def main() -> None:
    """Emit one JSON result per recording; unfinished DSP remains visible in status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('recordings', nargs='+', type=Path)
    arguments = parser.parse_args()
    config = load_config(arguments.config)
    pipeline = Pipeline(config)
    for path in arguments.recordings:
        try:
            block = load_saleae_block(path, config.geometry.channel_ids)
            result = json_value(pipeline.process(block))
        except (ValueError, KeyError, OSError) as error:
            result = {'status': 'invalid', 'reason': str(error)}
        print(json.dumps({'recording': str(path), 'result': result}, allow_nan=False))


if __name__ == '__main__':
    main()
