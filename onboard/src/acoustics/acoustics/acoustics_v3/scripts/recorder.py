"""Data controller for batch hydrophone data collection."""
import os
import time
from logic import logic


def collect_batch_data(
        sampling_freq,
        epochs,
        capture_time,
        output_base_path,
        test_name,
        is_logic_2=False,
        is_mock=False
        ):
    """Collect multiple epochs of hydrophone data.

    Args:
        sampling_freq: Sampling frequency in Hz
        epochs: Number of data collection runs
        capture_time: Duration of each capture in seconds
        output_base_path: Base directory for saving data
        test_name: Name for this test session
        is_logic_2: Whether to use Logic 2 (True) or Logic 1 (False)
        is_mock: Whether to use mock device for Logic 2 (True) or real device (False)

    Returns:
        Path to test directory containing all epochs
    """
    # Create test directory with timestamp
    timestamp = time.strftime('%Y-%m-%d--%H-%M-%S')
    test_folder = f"{test_name}_{timestamp}" if test_name else timestamp
    test_path = os.path.join(output_base_path, test_folder)
    os.makedirs(test_path, exist_ok=True)

    # Initialize Logic interface
    if is_logic_2:
        from logic.logic2 import Logic2
        logic_interface = Logic2(is_mock=is_mock)
    else:
        logic_interface = logic.Logic(sampling_freq=sampling_freq)
        logic_interface.print_saleae_status()

    # Collect data for each epoch
    for epoch in range(epochs):
        print(f"\nEpoch {epoch}/{epochs}")
        capture_name = f"{test_name}_epoch_{epoch}" if test_name else f"epoch_{epoch}"
        
        if is_logic_2:
            logic_interface.capture(
                seconds=capture_time,
                prefix=capture_name,
                base_dir=test_path,
                sample_rate=int(sampling_freq),
                formats=["bin"]
            )
        else:
            logic_interface.export_binary_capture(capture_time, test_path, capture_name)

    logic_interface.close() if is_logic_2 else logic_interface.kill_logic()

    print("\n" + "=" * 60)
    print(f"Collection complete! Total epochs: {epochs}")
    print(f"Data location: {test_path}")
    print("=" * 60)

    return test_path


if __name__ == "__main__":
    # ==================== CONFIGURATION ====================

    # Whether to use Logic 2 or Logic 1
    IS_LOGIC_2 = False

    # Whether to use mock device for Logic 2 (True) or real device (False)
    USE_MOCK_DEVICE = True

    # Name for this test (0,1,2,3)
    TEST_NAME = "0"

    # Number of capture epochs to collect
    EPOCHS = 100

    # Sampling frequency in Hz for data acquisition
    SAMPLING_FREQ = 781250

    # Duration of each capture in seconds
    CAPTURE_TIME = 2

    # Base directory for saving captured data
    OUTPUT_PATH = "Temp_Data"

    # ==================== EXECUTION ====================

    test_directory = collect_batch_data(
        sampling_freq=SAMPLING_FREQ,
        epochs=EPOCHS,
        capture_time=CAPTURE_TIME,
        output_base_path=OUTPUT_PATH,
        test_name=TEST_NAME,
        is_logic_2=IS_LOGIC_2,
        is_mock=USE_MOCK_DEVICE
    )
