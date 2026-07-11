"""Logic module for interfacing with Saleae Logic hardware."""
import time
import os
import sys

import saleae


class Logic():
    """Interface for Saleae Logic data acquisition hardware."""
    def __init__(self, sampling_freq=781250, logic_path=""):
        self._launch_timeout = 15
        self._quiet = False
        self._port = 10429
        self._host = 'localhost'

        if logic_path != "":
            self._logic_path = logic_path
        elif sys.platform == "win32":
            self._logic_path = "logic_software/Logic-1.2.40_WINDOWS/Logic.exe"
        elif sys.platform == "darwin":
            self._logic_path = "Logic-1.2.40-MacOS.dmg"
        elif sys.platform == "linux":
            self._logic_path = "logic_software/Logic-1.2.40-Linux.AppImage"
        else:
            print(f"Unknown OS: {sys.platform}")

        self._device_selection = 1    # 0 for LOGIC PRO 16, 1 for LOGIC 8, 2 for LOGIC PRO 8
        self._sampling_freq = sampling_freq
        self._h0_channel = 0
        self._h1_channel = 1
        self._h2_channel = 2
        self._h3_channel = 3
        channels = [self._h0_channel, self._h1_channel,
                    self._h2_channel, self._h3_channel]
        self._channels = channels

        if sys.platform != "linux":
            self.start_logic()
        self._saleae = saleae.Saleae(
            host=self._host, port=self._port, quiet=self._quiet
        )
        self._configure_device()

    def start_logic(self):
        """Start the Logic software if not already running."""
        if not saleae.Saleae.is_logic_running():
            return saleae.Saleae.launch_logic(
                timeout=self._launch_timeout,
                quiet=self._quiet,
                host=self._host,
                port=self._port,
                logic_path=self._logic_path
            )
        return True

    def kill_logic(self):
        """Kill the Logic software process."""
        saleae.Saleae.kill_logic()

    def _configure_device(self):
        """Configure the active Logic device with channels and sample rate."""
        self._saleae.select_active_device(self._device_selection)
        self._saleae.set_active_channels(digital=None, analog=self._channels)
        self._saleae.set_sample_rate_by_minimum(0, self._sampling_freq)

    def print_saleae_status(self):
        """Print debug information about the Saleae Logic device status."""
        print(f"DEBUG: IS LOGIC RUNNING: {self._saleae.is_logic_running()}")
        print(f"DEBUG: CONNECTED DEVICE: {self._saleae.get_connected_devices()}")
        print(f"DEBUG: PERFORMANCE: {self._saleae.get_performance()}")
        print(f"DEBUG: ACTIVE CHANNELS: {self._saleae.get_active_channels()}")
        rates = self._saleae.get_all_sample_rates()
        print(f"DEBUG: POSSIBLE SAMPLING RATES: {rates}")
        print(f"DEBUG: SAMPLING RATE: {self._saleae.get_sample_rate()}")
        bandwidth = self._saleae.get_bandwidth(self._saleae.get_sample_rate())
        print(f"DEBUG: POSSIBLE BANDWIDTH: {bandwidth}")
        print(f"DEBUG: ANALYZERS: {self._saleae.get_analyzers()}")

    def start_csv_capture(self, seconds, output_dir):
        """Capture data and export to CSV format."""
        csv_path = os.path.join(output_dir, "TEMP.csv")
        self._saleae.set_capture_seconds(seconds)
        self._saleae.capture_start_and_wait_until_finished()
        self._saleae.export_data2(
            file_path_on_target_machine=csv_path, format='csv'
        )
        while not self._saleae.is_processing_complete():
            time.sleep(0.5)
        return csv_path

    def export_binary_capture(self, seconds, output_dir, name="TEMP.bin"):
        """Capture data and export to binary format."""
        bin_path = os.path.join(output_dir, name)
        self._saleae.set_capture_seconds(seconds)
        self._saleae.capture_start_and_wait_until_finished()
        self._saleae.export_data2(
            file_path_on_target_machine=bin_path,
            analog_channels=self._channels,
            format='binary'
        )
        while not self._saleae.is_processing_complete():
            time.sleep(0.5)
        return bin_path

    def export_binary_and_csv_capture(self, seconds, output_dir):
        """Capture data and export to both binary and CSV formats."""
        bin_path = os.path.join(output_dir, "TEMP.bin")
        csv_path = os.path.join(output_dir, "TEMP.csv")

        self._saleae.set_capture_seconds(seconds)
        self._saleae.capture_start_and_wait_until_finished()
        self._saleae.export_data2(
            file_path_on_target_machine=bin_path,
            analog_channels=self._channels,
            format='binary'
        )
        while not self._saleae.is_processing_complete():
            time.sleep(0.5)

        self._saleae.export_data2(
            file_path_on_target_machine=csv_path, format='csv'
        )
        while not self._saleae.is_processing_complete():
            time.sleep(0.5)

        return bin_path, csv_path
