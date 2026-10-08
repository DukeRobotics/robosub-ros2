# ruff: noqa
# Copied upstream source; packaging import only changed. See upstream/README.md for provenance.
"""Hydrophone array module for multi-sensor data processing."""
from __future__ import annotations

import os
import struct
from typing import BinaryIO

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

from . import hydrophone as hydrophone_module


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
        """Load a Logic 2 capture directory."""
        if not os.path.isdir(path):
            raise ValueError(f"Expected a Logic 2 capture directory: {path}")
        self._load_from_logic2_directory(path)

    def _update_hydrophone(self, hydro, times, signal):
        hydro.times = times
        hydro.signal = signal - np.mean(signal)
        
        # Note: FFT computation is deferred to plot_hydrophones() for performance.
        # Analyzers compute their own FFT on filtered signals, not raw data.
        hydro.freqs = None
        hydro.frequency = None

    def _reset_hydrophones(self):
        for hydro in self.hydrophones:
            hydro.reset()

    def _parse_analog_v1(self, f: BinaryIO) -> list:
        """Parse Logic 2 analog binary format version 0 (Saleae format).
        
        Returns:
            List of waveform dictionaries with keys: begin_time, trigger_time, sample_rate, downsample, num_samples, samples
        """
        # Parse header
        identifier = f.read(8)
        if identifier != b"<SALEAE>":
            raise ValueError("Not a Saleae file")

        version, datatype = struct.unpack('<ii', f.read(8))

        if datatype != 1:  # TYPE_ANALOG
            raise ValueError(f"Expected analog data, got type {datatype}")

        if version != 0:
            raise ValueError(f"Expected version 0, got version {version}")

        # Version 0 format - single waveform
        begin_time, sample_rate, downsample, num_samples = struct.unpack('<dQQQ', f.read(32))
        trigger_time = begin_time

        samples = [struct.unpack('<f', f.read(4))[0] for _ in range(num_samples)]

        waveforms = [{
            'begin_time': begin_time,
            'trigger_time': trigger_time,
            'sample_rate': sample_rate,
            'downsample': downsample,
            'num_samples': num_samples,
            'samples': samples
        }]

        return waveforms

    def _load_from_logic2_directory(self, directory: str) -> None:
        """Load Logic 2 analog data from a directory (CSV or binary files).
        
        For binary files, expects filenames with suffixes _0, _1, _2, _3 to identify hydrophone index.
        """
        self._reset_hydrophones()

        # First, check if there's a CSV file in the directory
        csv_file = None
        if os.path.isdir(directory):
            for filename in os.listdir(directory):
                if filename.endswith('.csv'):
                    csv_file = os.path.join(directory, filename)
                    break

        # Logic 2 can export either a combined CSV or per-channel binaries.
        if csv_file:
            self._load_logic2_csv(csv_file)
            print(f"Loaded Logic 2 data from CSV: {os.path.basename(csv_file)}")
            return

        # Otherwise, look for analog bin files with pattern _0, _1, _2, _3
        if os.path.isdir(directory):
            for filename in os.listdir(directory):
                if not filename.endswith('.bin'):
                    continue

                # Extract hydrophone index from filename suffix (e.g., _0.bin, _1.bin)
                base_name = filename[:-4]  # Remove .bin
                if base_name[-2] == '_' and base_name[-1].isdigit():
                    hydro_idx = int(base_name[-1])
                else:
                    continue

                analog_file = os.path.join(directory, filename)
                try:
                    with open(analog_file, 'rb') as f:
                        waveforms = self._parse_analog_v1(f)

                    if not waveforms:
                        continue

                    # Use first waveform
                    waveform = waveforms[0]
                    actual_sample_period = waveform['downsample'] / waveform['sample_rate']

                    # Generate time array
                    times = np.array([
                        waveform['begin_time'] + (i * actual_sample_period)
                        for i in range(len(waveform['samples']))
                    ], dtype=np.float64)

                    signal = np.array(waveform['samples'], dtype=np.float32)

                    hydro = self.hydrophones[hydro_idx]
                    hydro.sampling_period = actual_sample_period
                    self._update_hydrophone(hydro, times, signal)

                    print(f"Loaded Logic 2 channel {hydro_idx} from {os.path.basename(analog_file)}")

                except Exception as e:
                    print(f"Error loading {analog_file}: {e}")

    def _load_logic2_csv(self, path: str) -> None:
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
        sampling_period = (
            (times[-1] - times[0]) / (len(times) - 1)
            if len(times) > 1 else self.sampling_period
        )

        for idx, hydro in enumerate(self.hydrophones):
            hydro.sampling_period = sampling_period
            self._update_hydrophone(
                hydro, times, data.iloc[:, idx + 1].to_numpy()
            )


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

                # Frequency domain - compute FFT lazily only for plotting
                if hydro.frequency is None:
                    hydro.freqs = fftfreq(len(hydro.signal), hydro.sampling_period)
                    hydro.frequency = fft(hydro.signal)
                
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
