"""Garbage detector module for filtering invalid samples."""


class GarbageDetector:
    """Simple utility for validating individual TOA measurements."""

    def __init__(
        self,
        raw_signal_threshold: float = 3,
        margin_front: float = 0.1,
        margin_end: float = 0.1
    ):
        """Initialize garbage detector.
        
        Args:
            raw_signal_threshold: Minimum absolute amplitude in raw signal
            margin_front: Minimum time (seconds) from recording start for valid TOA
            margin_end: Minimum time (seconds) from recording end for valid TOA
        """
        self.raw_signal_threshold = raw_signal_threshold
        self.margin_front = margin_front
        self.margin_end = margin_end

    def validate_hydrophone_toa(
        self,
        signal_value: float,
        toa_time: float,
        recording_start: float,
        recording_end: float
    ) -> bool:
        """Validate a single hydrophone's TOA measurement.
        
        Args:
            signal_value: Raw signal value at TOA index
            toa_time: Time of arrival (seconds)
            recording_start: Start time of recording (seconds)
            recording_end: End time of recording (seconds)
            
        Returns:
            True if measurement is valid, False otherwise
        """
        # Check raw signal amplitude
        if abs(signal_value) < self.raw_signal_threshold:
            return False
        
        # Check timing margins
        if toa_time < recording_start + self.margin_front:
            return False
        if toa_time > recording_end - self.margin_end:
            return False
        
        return True
