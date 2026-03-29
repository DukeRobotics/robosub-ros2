"""TOA estimation using Hilbert envelope detection."""
import numpy as np
from scipy.signal import hilbert
from scipy.fft import fft, fftfreq

from .base_analyzer import BaseAnalyzer
from .garbage_detector import GarbageDetector


class TOAEnvelopeAnalyzer(BaseAnalyzer):
    """Time of Arrival (TOA) estimation using Hilbert envelope detection.
    
    This analyzer detects signal arrival times by computing the Hilbert envelope
    and finding the first point where it exceeds a threshold based on signal statistics.
    """

    def __init__(
        self,
        threshold_sigma=5,
        raw_signal_threshold=3,
        margin_front=0.1,
        margin_end=0.1,
        **kwargs
    ):
        """Initialize TOA envelope analyzer.
        
        Args:
            threshold_sigma: Number of standard deviations above mean for TOA threshold
            raw_signal_threshold: Minimum absolute amplitude in raw signal for validation
            margin_front: Minimum time (seconds) from recording start for valid TOA
            margin_end: Minimum time (seconds) from recording end for valid TOA
            **kwargs: Additional arguments passed to BaseAnalyzer
        """
        super().__init__(**kwargs)
        self.threshold_sigma = threshold_sigma
        self.garbage_detector = GarbageDetector(
            raw_signal_threshold=raw_signal_threshold,
            margin_front=margin_front,
            margin_end=margin_end
        )

    def get_name(self):
        """Return analyzer name.
        
        Returns:
            String identifier for this analyzer
        """
        return "TOA Envelope Detection"

    def print_results(self, analysis_results):
        """Print TOA detection results.
        
        Args:
            analysis_results: Dictionary returned from analyze_array
        """
        super().print_results(analysis_results)
        print(f"\nTOA Estimates:")
        for result in analysis_results['results']:
            is_valid = result.get('is_valid', '?')
            print(f"  Hydrophone {result['hydrophone_idx']}: {result['toa_time']:.6f} s (sample {result['toa_idx']}) [Valid: {is_valid}]")

    def _analyze_single(self, hydrophone, sampling_freq):
        """Analyze single hydrophone using envelope detection.
        
        Args:
            hydrophone: Hydrophone object with signal data
            sampling_freq: Sampling frequency in Hz
            
        Returns:
            Dictionary containing:
                - toa_time: Detected time of arrival (seconds)
                - toa_idx: Sample index of TOA
                - filtered_signal: Bandpass filtered signal
                - processed_signal: Hilbert envelope of filtered signal
                - filtered_frequency: FFT of filtered signal
                - filtered_freqs: Frequency bins for FFT
                - threshold: Detection threshold value
                - band_min: Lower frequency bound used (Hz)
                - band_max: Upper frequency bound used (Hz)
                - is_valid: Whether sample passes garbage detection (bool)
        """
        # Apply bandpass filter
        filtered_signal = self.apply_bandpass(
            hydrophone.signal, sampling_freq
        )

        # Compute envelope using Hilbert transform
        envelope = np.abs(hilbert(filtered_signal))

        # Detect TOA using threshold
        threshold = (
            np.mean(envelope) +
            self.threshold_sigma * np.std(envelope)
        )
        toa_candidates = np.where(envelope > threshold)[0]
        if len(toa_candidates) > 0:
            toa_idx = toa_candidates[0]  # First crossing
        else:
            toa_idx = np.argmax(envelope)  # Fallback to peak

        toa_time = hydrophone.times[toa_idx]

        # Compute filtered frequency spectrum
        filtered_frequency = fft(filtered_signal)
        filtered_freqs = fftfreq(len(filtered_signal), 1/sampling_freq)

        # Validate sample using garbage detector
        is_valid = self.garbage_detector.validate_hydrophone_toa(
            signal_value=np.max(hydrophone.signal),
            toa_time=toa_time,
            recording_start=hydrophone.times[0],
            recording_end=hydrophone.times[-1]
        )

        return {
            'toa_time': toa_time,
            'toa_idx': toa_idx,
            'filtered_signal': filtered_signal,
            'processed_signal': envelope,
            'filtered_frequency': filtered_frequency,
            'filtered_freqs': filtered_freqs,
            'threshold': threshold,
            'band_min': self.search_band_min,
            'band_max': self.search_band_max,
            'is_valid': is_valid
        }

    def _plot_single_signal(self, ax_time, ax_freq, hydrophone, result, idx):
        """Plot envelope analysis results for a single hydrophone.
        
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
        ax_time.plot(
            hydrophone.times, result['processed_signal'],
            label='Envelope', color='darkblue', linewidth=2
        )
        ax_time.axhline(
            result['threshold'], color='green',
            linestyle=':', alpha=0.5, label='Threshold'
        )
        ax_time.axvline(
            result['toa_time'], color='red',
            linestyle='--', linewidth=2,
            label=f"TOA: {result['toa_time']:.6f}s"
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
