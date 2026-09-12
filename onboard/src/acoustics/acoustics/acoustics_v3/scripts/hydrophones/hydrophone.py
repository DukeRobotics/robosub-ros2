"""Hydrophone module for storing sensor data."""

import numpy as np


class Hydrophone:
    """Data container for a single hydrophone sensor."""
    times: np.ndarray | None = None
    signal: np.ndarray | None = None
    filtered_signal: np.ndarray | None = None

    freqs: np.ndarray | None = None
    frequency: np.ndarray | None = None
    filtered_frequency: np.ndarray | None = None

    # Hydrophone-specific sampling parameters
    sampling_period: float | None = None  # Time between samples in seconds

    def reset(self) -> None:
        """Reset all data arrays and metadata to None."""
        self.times = None
        self.signal = None
        self.filtered_signal = None
        self.freqs = None
        self.frequency = None
        self.filtered_frequency = None
        self.sampling_period = None
