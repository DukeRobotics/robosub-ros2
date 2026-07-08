"""Hydrophone module for storing sensor data."""
from typing import Optional
import numpy as np


class Hydrophone:
    """Data container for a single hydrophone sensor."""
    times: Optional[np.ndarray] = None
    signal: Optional[np.ndarray] = None
    filtered_signal: Optional[np.ndarray] = None

    freqs: Optional[np.ndarray] = None
    frequency: Optional[np.ndarray] = None
    filtered_frequency: Optional[np.ndarray] = None

    # Hydrophone-specific sampling parameters
    sampling_period: Optional[float] = None  # Time between samples in seconds

    def reset(self):
        """Reset all data arrays and metadata to None."""
        self.times = None
        self.signal = None
        self.filtered_signal = None
        self.freqs = None
        self.frequency = None
        self.filtered_frequency = None
        self.sampling_period = None
