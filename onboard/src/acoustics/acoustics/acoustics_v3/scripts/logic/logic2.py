"""Logic 2 module for interfacing with Saleae Logic 2 hardware."""
from pathlib import Path

from saleae.automation import CaptureConfiguration, LogicDeviceConfiguration, Manager, TimedCaptureMode
from saleae.automation.errors import Logic2AlreadyRunningError


class Logic2:
    """Interface for Saleae Logic 2 data acquisition hardware."""

    def __init__(self, is_mock: bool = False) -> None:
        app_path = ('/home/ubuntu/robosub-ros2/onboard/src/acoustics/acoustics/acoustics_v3/'
                    'Logic-2.4.40-linux-x64.AppImage')
        try:
            self._manager = Manager.launch(application_path=app_path)  # Use default path to Logic 2
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

    def capture(self, seconds: float, prefix: str, base_dir: str, sample_rate: float = 781250,
               formats: list[str] | None = None) -> dict[str, str]:
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
        output_dir = (Path(base_dir) / prefix).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)

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
            capture.export_raw_data_csv(directory=str(output_dir))
            results['csv'] = str(output_dir / 'data.csv')

        if 'bin' in formats or formats == ['csv', 'bin']:
            capture.export_raw_data_binary(directory=str(output_dir))
            results['bin'] = str(output_dir)

        capture.close()
        return results

if __name__ == '__main__':
    logic = Logic2(is_mock=False)
    print(logic.capture(2,'TEST_2','/home/ubuntu/robosub-ros2/onboard/src/acoustics/acoustics/acoustics_v3/Temp_Data/TEST_2'))
    logic.close()
