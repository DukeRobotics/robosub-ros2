"""Hydrophone array module for multi-sensor data processing."""
from __future__ import annotations

import os
import struct

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

from hydrophones import hydrophone as hydrophone_module


class HydrophoneArray:
    """Array of hydrophone sensors with signal processing capabilities."""
    def __init__(
        self,
        sampling_freq: float = 781250,
        selected: list[bool] | None = None
    ):
        self.sampling_freq = sampling_freq
        self.sampling_period = 1 / sampling_freq

        self.selected = selected if selected is not None else [True] * 4

        self.hydrophones = [
            hydrophone_module.Hydrophone(),
            hydrophone_module.Hydrophone(),
            hydrophone_module.Hydrophone(),
            hydrophone_module.Hydrophone()
        ]

    def load_from_path(self, path: str) -> None:
        """Load hydrophone data from a file (bin or csv format)."""
        ext = os.path.splitext(path)[1].lower()
        if ext == ".bin":
            self._load_from_bin(path)
        elif ext == ".csv":
            self._load_from_csv(path)
        else:
            raise ValueError(f"Unsupported file type: {ext}. Expected .bin or .csv")

    def _load_from_csv(self, path: str) -> None:
        self._reset_hydrophones()

        skip_rows = 0
        with open(path, 'r', encoding='utf-8') as f:
            for i, line in enumerate(f):
                parts = line.strip().split(',')
                try:
                    float(parts[0])
                    skip_rows = i
                    break
                except ValueError:
                    continue

        data = pd.read_csv(path, skiprows=skip_rows, header=None)

        times = data.iloc[:, 0].to_numpy()
        for idx, hydro in enumerate(self.hydrophones):
            self._update_hydrophone(
                hydro, times, data.iloc[:, idx + 1].to_numpy()
            )

    def _load_from_bin(self, path: str) -> None:
        self._reset_hydrophones()

        with open(path, "rb") as f:
            # Read header: 8 bytes uint64, 4 bytes uint32, 8 bytes double
            header = f.read(8 + 4 + 8)
            num_samples, num_channels, sample_period = struct.unpack(
                "<QId", header
            )
            if sample_period:
                self.sampling_period = sample_period
            # read all float32 samples
            total_floats = num_samples * num_channels
            float_bytes = f.read(total_floats * 4)
            data = np.frombuffer(float_bytes, dtype="<f4")
            data = data.reshape((num_channels, num_samples))

        times = np.arange(num_samples, dtype=np.float64) * self.sampling_period
        for idx, hydro in enumerate(self.hydrophones):
            self._update_hydrophone(hydro, times, data[idx])

    def _update_hydrophone(self, hydro, times, signal):
        hydro.times = times
        hydro.signal = signal - np.mean(signal)
        hydro.freqs = fftfreq(len(hydro.signal), self.sampling_period)
        hydro.frequency = fft(hydro.signal)

    def _reset_hydrophones(self):
        for hydro in self.hydrophones:
            hydro.reset()

    def plot_hydrophones(self):
        """Plot basic hydrophone data: signal and frequency."""
        num_plots = sum(self.selected)

        _, axes = plt.subplots(num_plots, 2, figsize=(14, 3*num_plots), squeeze=False)

        plot_idx = 0
        for i, (hydro, is_selected) in enumerate(
            zip(self.hydrophones, self.selected)
        ):
            if is_selected:
                # Time domain - signal
                axes[plot_idx, 0].plot(hydro.times, hydro.signal, color='blue')
                axes[plot_idx, 0].set_ylabel('Amplitude')
                axes[plot_idx, 0].set_title(f'Hydrophone {i} - Signal')
                axes[plot_idx, 0].grid(True, alpha=0.3)

                # Frequency domain
                freq_mask = hydro.freqs >= 0
                freqs = hydro.freqs[freq_mask]
                magnitude = np.abs(hydro.frequency[freq_mask])

                axes[plot_idx, 1].plot(freqs, magnitude, color='blue')
                axes[plot_idx, 1].set_ylabel('Magnitude')
                axes[plot_idx, 1].set_title(f'Hydrophone {i} - Frequency')
                axes[plot_idx, 1].grid(True, alpha=0.3)
                axes[plot_idx, 1].set_xlim([0, 100000])

                plot_idx += 1

        axes[-1, 0].set_xlabel('Time (s)')
        axes[-1, 1].set_xlabel('Frequency (Hz)')
        plt.tight_layout()
        plt.show()
