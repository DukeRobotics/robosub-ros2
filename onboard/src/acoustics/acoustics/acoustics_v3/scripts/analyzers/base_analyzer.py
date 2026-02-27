"""Base analyzer module for hydrophone signal processing."""
from abc import ABC, abstractmethod
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfilt


class BaseAnalyzer(ABC):
    """Base class for hydrophone signal analyzers.
    
    This abstract class provides common filtering and analysis infrastructure
    for different hydrophone signal processing algorithms.
    """

    def __init__(
        self,
        search_band_min: float = 25000,
        search_band_max: float = 40000,
        filter_order: int = 8,
        plot_results: bool = False,
        config: dict | None = None
    ):
        """Initialize analyzer with signal processing parameters.
        
        Args:
            search_band_min: Lower frequency bound for analysis (Hz)
            search_band_max: Upper frequency bound for analysis (Hz)
            filter_order: Order of Butterworth bandpass filter
            plot_results: Whether to plot analysis results
            config: Optional configuration dictionary
        """
        self.search_band_min = search_band_min
        self.search_band_max = search_band_max
        self.filter_order = filter_order
        self.plot_results_flag = plot_results
        self.config = config or {}

    # ==================== ABSTRACT METHODS ====================

    @abstractmethod
    def _analyze_single(self, hydrophone, sampling_freq) -> dict:
        """Analyze a single hydrophone signal.
        
        Args:
            hydrophone: Hydrophone object with signal data
            sampling_freq: Sampling frequency in Hz
            
        Returns:
            Dictionary containing analysis results with keys:
                - toa_time: Time of arrival (float)
                - toa_idx: Index of time of arrival (int)
                - filtered_signal: Bandpass filtered signal (np.ndarray)
                - processed_signal: Post-processed signal (np.ndarray)
                - Additional analyzer-specific fields
        """

    @abstractmethod
    def _plot_single_signal(self, ax_time, ax_freq, hydrophone, result, idx):
        """Plot analysis results for a single hydrophone.
        
        Args:
            ax_time: Matplotlib axis for time domain plot
            ax_freq: Matplotlib axis for frequency domain plot
            hydrophone: Hydrophone object with signal data
            result: Analysis result dictionary from _analyze_single
            idx: Hydrophone index
        """

    @abstractmethod
    def get_name(self) -> str:
        """Return the name of this analyzer.
        
        Returns:
            String identifier for the analyzer
        """

    # ==================== PUBLIC ====================

    def analyze_array(self, hydrophone_array, selected: list[bool] | None = None):
        """Analyze all selected hydrophones in the array.
        
        Args:
            hydrophone_array: HydrophoneArray object containing sensor data
            selected: List of booleans indicating which hydrophones to analyze
            
        Returns:
            Dictionary with keys:
                - results: List of individual hydrophone analysis results
                - analyzer: Name of the analyzer
        """
        if selected is None:
            selected = hydrophone_array.selected

        # Analyze each hydrophone
        results = []
        for idx, (hydro, is_selected) in enumerate(
            zip(hydrophone_array.hydrophones, selected)
        ):
            if is_selected:
                result = self._analyze_single(
                    hydro, hydrophone_array.sampling_freq
                )
                result['hydrophone_idx'] = idx
                results.append(result)

        analysis_results = {
            'results': results,
            'analyzer': self.get_name()
        }

        if self.plot_results_flag:
            self.plot_results(hydrophone_array, analysis_results, selected)

        return analysis_results

    def print_results(self, analysis_results):
        """Print analysis results to console.
        
        Args:
            analysis_results: Dictionary returned from analyze_array
        """
        print(f"\n{analysis_results['analyzer']}")

    def plot_results(self, hydrophone_array, analysis_results, selected=None):
        """Plot analysis results for all hydrophones.
        
        Args:
            hydrophone_array: HydrophoneArray object containing sensor data
            analysis_results: Dictionary returned from analyze_array
            selected: List of booleans indicating which hydrophones were analyzed
        """
        if selected is None:
            selected = hydrophone_array.selected

        results = analysis_results['results']
        num_plots = len(results)

        # Create 2 columns: time domain and frequency domain
        _, axes = plt.subplots(
            num_plots, 2, figsize=(14, 3*num_plots), squeeze=False
        )

        for plot_idx, result in enumerate(results):
            hydro_idx = result['hydrophone_idx']
            hydro = hydrophone_array.hydrophones[hydro_idx]

            # Each analyzer defines how to plot ONE signal
            self._plot_single_signal(
                axes[plot_idx, 0], axes[plot_idx, 1],
                hydro, result, hydro_idx
            )

            # Common formatting
            axes[plot_idx, 0].set_ylabel('Amplitude')
            axes[plot_idx, 0].set_title(f'Hydrophone {hydro_idx} - Time Domain')
            axes[plot_idx, 0].legend(loc='best')
            axes[plot_idx, 0].grid(True, alpha=0.3)

            axes[plot_idx, 1].set_ylabel('Magnitude')
            axes[plot_idx, 1].set_title(f'Hydrophone {hydro_idx} - Frequency Domain')
            axes[plot_idx, 1].legend(loc='best')
            axes[plot_idx, 1].grid(True, alpha=0.3)

        axes[-1, 0].set_xlabel('Time (s)')
        axes[-1, 1].set_xlabel('Frequency (Hz)')
        plt.tight_layout()
        plt.show()

    # ==================== COMMON ====================

    def apply_bandpass(self, signal, sampling_freq, band_min=None, band_max=None):
        """Apply Butterworth bandpass filter to signal.
        
        Args:
            signal: Input signal array
            sampling_freq: Sampling frequency in Hz
            band_min: Lower frequency bound (uses search_band_min if None)
            band_max: Upper frequency bound (uses search_band_max if None)
            
        Returns:
            Filtered signal array
        """
        if band_min is None:
            band_min = self.search_band_min
        if band_max is None:
            band_max = self.search_band_max

        sos = butter(
            self.filter_order,
            [band_min, band_max],
            fs=sampling_freq,
            btype='band',
            output='sos'
        )
        return sosfilt(sos, signal)


