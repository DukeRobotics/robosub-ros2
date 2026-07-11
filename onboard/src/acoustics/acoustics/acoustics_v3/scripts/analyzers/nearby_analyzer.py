"""Nearby detection using static threshold analysis."""
import numpy as np
from scipy.fft import fft, fftfreq

from .base_analyzer import BaseAnalyzer


class NearbyAnalyzer(BaseAnalyzer):
    """Nearby presence detection using static threshold analysis.
    
    This analyzer determines if a signal source is nearby by checking if the
    filtered signal exceeds a static amplitude threshold.
    """

    def __init__(self, threshold, **kwargs):
        """Initialize nearby analyzer.
        
        Args:
            threshold: Static amplitude threshold for nearby detection
            **kwargs: Additional arguments passed to BaseAnalyzer
        """
        super().__init__(**kwargs)
        self.threshold = threshold

    def get_name(self):
        """Return analyzer name.
        
        Returns:
            String identifier for this analyzer
        """
        return "Static Nearby Analyzer"

    def print_results(self, analysis_results):
        """Print nearby detection results.
        
        Args:
            analysis_results: Dictionary returned from analyze_array
        """
        super().print_results(analysis_results)
        print(f"\nNearby Detection (threshold: {self.threshold}):")
        for result in analysis_results['results']:
            status = "NEARBY" if result['nearby'] else "NOT NEARBY"
            print(f"  Hydrophone {result['hydrophone_idx']}: {status}")

    def _analyze_single(self, hydrophone, sampling_freq):
        """Analyze single hydrophone using static threshold.
        
        Args:
            hydrophone: Hydrophone object with signal data
            sampling_freq: Sampling frequency in Hz
            
        Returns:
            Dictionary containing:
                - nearby: Boolean indicating if signal exceeds threshold
                - filtered_signal: Bandpass filtered signal
                - filtered_frequency: FFT of filtered signal
                - filtered_freqs: Frequency bins for FFT
                - threshold: Detection threshold value
                - band_min: Lower frequency bound used (Hz)
                - band_max: Upper frequency bound used (Hz)
        """
        # Apply bandpass filter
        filtered_signal = self.apply_bandpass(
            hydrophone.signal, sampling_freq
        )

        # Detect threshold crossings
        toa_candidates = np.where(filtered_signal > self.threshold)[0]
        nearby = len(toa_candidates) > 0

        # Compute filtered frequency spectrum
        filtered_frequency = fft(filtered_signal)
        filtered_freqs = fftfreq(len(filtered_signal), 1/sampling_freq)

        return {
            'nearby': nearby,
            'filtered_signal': filtered_signal,
            'filtered_frequency': filtered_frequency,
            'filtered_freqs': filtered_freqs,
            'threshold': self.threshold,
            'band_min': self.search_band_min,
            'band_max': self.search_band_max
        }

    def _plot_single_signal(self, ax_time, ax_freq, hydrophone, result, idx):
        """Plot nearby detection results for a single hydrophone.
        
        Args:
            ax_time: Matplotlib axis for time domain plot
            ax_freq: Matplotlib axis for frequency domain plot
            hydrophone: Hydrophone object with signal data
            result: Analysis result dictionary from _analyze_single
            idx: Hydrophone index
        """
        # Time domain plot
        ax_time.plot(
            hydrophone.times, result['filtered_signal'],
            alpha=0.5, label='Filtered Signal', color='blue'
        )
        ax_time.axhline(
            result['threshold'], color='green',
            linestyle=':', alpha=0.5, label='Threshold'
        )

        # Indicate if nearby
        status = 'NEARBY' if result['nearby'] else 'NOT NEARBY'
        color = 'green' if result['nearby'] else 'red'
        ax_time.text(
            0.5, 0.95, status,
            transform=ax_time.transAxes,
            fontsize=12, fontweight='bold',
            color=color, ha='center', va='top'
        )

        # Frequency domain plot
        freq_mask = result['filtered_freqs'] >= 0
        freqs = result['filtered_freqs'][freq_mask]
        magnitude = np.abs(result['filtered_frequency'][freq_mask])

        ax_freq.plot(freqs, magnitude, label='Filtered Spectrum', color='blue')
        ax_freq.axvline(
            result['band_min'], color='red',
            linestyle='--', alpha=0.5, label='Filter Range'
        )
        ax_freq.axvline(
            result['band_max'], color='red',
            linestyle='--', alpha=0.5
        )
        ax_freq.set_xlim([0, 100000])  # Focus on relevant frequency range
