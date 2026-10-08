# ruff: noqa
# Copied upstream source; packaging import only changed. See upstream/README.md for provenance.


import joblib
import numpy as np
from scipy.signal import hilbert
from scipy.fft import fft, fftfreq
import pandas as pd
from .base_analyzer import BaseAnalyzer


class NearbyAnalyzer(BaseAnalyzer):
    """ML-based nearby detection (≤10ft) using 3 key features."""

    def __init__(self, model_path: str, **kwargs):
        super().__init__(**kwargs)
        pkg = joblib.load(model_path)
        self.model = pkg['model']
        if hasattr(self.model, 'n_jobs'):
            self.model.n_jobs = 1
        self.features = pkg['features']  # Feature names in order
        print(f"Model features: {self.features}")

    def get_name(self):
        return "ML-based Nearby Detection (10ft)"

    def _analyze_single(self, hydrophone, sampling_freq):
        """Extract 3 features and predict if nearby."""
        signal = hydrophone.signal
        filtered = self.apply_bandpass(signal, sampling_freq)
        
        # Compute raw envelope for rise-time extraction.
        env_raw = np.abs(hilbert(signal))
        peak_raw = np.argmax(env_raw)
        
        # Extract the three features used by the model.
        flatness = self._spectral_flatness(signal)
        centroid = self._spectral_centroid(filtered, sampling_freq)
        rise_time = self._rise_time(env_raw, sampling_freq, peak_raw)
        
        # Prepare for model
        X = pd.DataFrame(
            [[flatness, centroid, rise_time]],
            columns=self.features
        )
        pred = self.model.predict(X)[0]
        prob = np.max(self.model.predict_proba(X)[0])
        
        return {
            'is_nearby': bool(pred),
            'confidence': float(prob),
            'feature_values': {
                'RAW_spectral_flatness': flatness,
                'FILTERED_spectral_centroid_hz': centroid,
                'RAW_rise_time_ms': rise_time,
            }
        }

    def _spectral_flatness(self, sig):
        spec = np.abs(fft(sig))
        geom = np.exp(np.mean(np.log(spec + 1e-10)))
        arith = np.mean(spec)
        return geom / (arith + 1e-10)

    def _spectral_centroid(self, sig, fs):
        spec = np.abs(fft(sig))
        freqs = fftfreq(len(sig), 1/fs)
        pos = freqs >= 0
        return np.sum(freqs[pos] * spec[pos]) / (np.sum(spec[pos]) + 1e-10)

    def _rise_time(self, env, fs, peak):
        thresh = env[peak] * 0.1
        i_start = next((i for i in range(peak, -1, -1) if env[i] < thresh), peak)
        return (peak - i_start) / fs * 1000

    def _plot_single_signal(self, ax_time, ax_freq, hydrophone, result, idx):
        """Plot signal and frequency content for a nearby/far prediction."""
        signal = hydrophone.signal
        filtered = self.apply_bandpass(signal, 1 / hydrophone.sampling_period)

        # Time-domain plot
        ax_time.plot(hydrophone.times, signal, alpha=0.5, label='Raw Signal', color='gray')
        ax_time.plot(hydrophone.times, filtered, label='Filtered Signal', color='blue', linewidth=2)

        nearby_label = "NEARBY" if result['is_nearby'] else "FAR"
        ax_time.set_title(f"H{idx} - {nearby_label} ({result['confidence']:.1%})")
        ax_time.set_ylabel('Amplitude')
        ax_time.grid(True, alpha=0.3)

        # Frequency-domain plot
        sample_period = hydrophone.sampling_period
        freqs = np.fft.rfftfreq(len(signal), d=sample_period)
        raw_mag = np.abs(np.fft.rfft(signal))
        filt_mag = np.abs(np.fft.rfft(filtered))

        ax_freq.plot(freqs, raw_mag, alpha=0.5, label='Raw FFT', color='gray')
        ax_freq.plot(freqs, filt_mag, label='Filtered FFT', color='blue', linewidth=2)
        ax_freq.set_xlim([0, 100000])
        ax_freq.set_ylabel('Magnitude')
        ax_freq.set_xlabel('Frequency (Hz)')
        ax_freq.grid(True, alpha=0.3)

    def print_results(self, analysis_results):
        """Print prediction for each hydrophone."""
        print(f"\n{analysis_results['analyzer']}")
        for r in analysis_results['results']:
            label = "NEARBY" if r['is_nearby'] else "FAR"
            print(f"  H{r.get('hydrophone_idx')}: {label} ({r['confidence']:.1%})")
