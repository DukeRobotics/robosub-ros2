"""Logic 2 module for interfacing with Saleae Logic 2 hardware."""
import os

from saleae.automation import CaptureConfiguration, LogicDeviceConfiguration, Manager, TimedCaptureMode
from saleae.automation.errors import Logic2AlreadyRunningError


class Logic2:
    """Interface for Saleae Logic 2 data acquisition hardware."""

    def __init__(self, is_mock=False) -> None:
        try:
            self._manager = Manager.launch(application_path='/home/ubuntu/robosub-ros2/onboard/src/acoustics/acoustics/acoustics_v3/Logic-2.4.40-linux-x64.AppImage')  # Use default path to Logic 2
        except Logic2AlreadyRunningError:
            # Manager already running, connect to existing instance
            self._manager = Manager.connect()

        devices = self._manager.get_devices(include_simulation_devices=is_mock)

        if not devices:
            self.close()
            msg = 'No Logic 2 devices found'
            raise RuntimeError(msg)

        if is_mock:
            self._device_id = 'F4244'
        else:
            self._device_id = devices[0].device_id

    def close(self) -> None:
        """Close the Logic 2 manager."""
        self._manager.close()

    def capture(self, seconds, prefix, base_dir, sample_rate=781250, formats=None):
        """
        Capture data and export to specified formats.

        Args:
            seconds: Duration of capture in seconds
            prefix: Directory name and file prefix for outputs
            base_dir: Base directory path (default: current directory)
            sample_rate: Sample rate in Hz (default: 781250)
            formats: List of formats to export ["csv", "bin", or both]

        Returns:
            dict with paths to exported files
        """
        # Create output directory with absolute path
        if formats is None:
            formats = ['csv', 'bin']
        output_dir = os.path.abspath(os.path.join(base_dir, prefix))
        os.makedirs(output_dir, exist_ok=True)

        # Create device configuration with sample rate and channels
        device_config = LogicDeviceConfiguration(
            enabled_analog_channels=[0, 1, 2, 3],
            analog_sample_rate=sample_rate,
        )

        # Start capture
        capture = self._manager.start_capture(
            device_id=self._device_id,
            device_configuration=device_config,
            capture_configuration=CaptureConfiguration(
                capture_mode=TimedCaptureMode(duration_seconds=seconds),
            ),
        )

        # Wait for completion
        capture.wait()

        # Export formats
        results = {}
        if 'csv' in formats or formats == ['csv', 'bin']:
            capture.export_raw_data_csv(directory=output_dir)
            results['csv'] = os.path.join(output_dir, 'data.csv')

        if 'bin' in formats or formats == ['csv', 'bin']:
            capture.export_raw_data_binary(directory=output_dir)
            results['bin'] = os.path.join(output_dir)

        capture.close()
        return results

if __name__ == '__main__':
    logic = Logic2(is_mock=False)
    print(logic.capture(2,'TEST_2','/home/ubuntu/robosub-ros2/onboard/src/acoustics/acoustics/acoustics_v3/Temp_Data/TEST_2'))
    logic.close()
