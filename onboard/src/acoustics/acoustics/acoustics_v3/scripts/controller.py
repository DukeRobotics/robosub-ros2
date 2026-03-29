"""Controller module for hydrophone data acquisition and analysis."""
import os
import time
from logic import logic
from logic.logic2 import Logic2
from hydrophones import hydrophone_array
from analyzers import TOAEnvelopeAnalyzer, NearbyAnalyzer


def run_controller(
        hydrophone_array,
        analyzers=None
        ):
    """Run analysis on hydrophone array.

    Args:
        hydrophone_array: HydrophoneArray with loaded data
        analyzers: Optional analyzer instance or list of analyzer instances to run

    Returns:
        Single analysis result dict if one analyzer, or list of dicts if multiple
    """
    if analyzers is None:
        return None
    
    results = []
    for analyzer in analyzers:
        print(f"\n{'='*60}")
        analysis_result = analyzer.analyze_array(hydrophone_array)
        analyzer.print_results(analysis_result)
        results.append(analysis_result)

    return results


def capture_data(
        sampling_freq,
        capture_time,
        capture_format,
        output_dir,
        is_logic_2=False,
        is_mock=False,
        close_logic_after=True
        ):
    """Capture new data from Logic hardware.

    Args:
        sampling_freq: Sampling frequency in Hz
        capture_time: Duration of capture in seconds
        capture_format: '.bin', '.csv', or 'both'
        output_dir: Directory to save capture files
        is_logic_2: Whether to use Logic 2 (True) or Logic 1 (False)
        is_mock: Whether to use mock device for Logic 2 (True) or real device (False)
        close_logic_after: Whether to close Logic after capture

    Returns:
        Path to captured data directory/file
    """
    os.makedirs(output_dir, exist_ok=True)

    if is_logic_2:
        logic_interface = Logic2(is_mock=is_mock)
        timestamp = time.strftime('%Y-%m-%d--%H-%M-%S')
        
        format_map = {".bin": ["bin"], ".csv": ["csv"], "both": ["csv", "bin"]}
        
        logic_interface.capture(
            seconds=capture_time,
            prefix=timestamp,
            base_dir=output_dir,
            sample_rate=int(sampling_freq),
            formats=format_map[capture_format]
        )
        
        if close_logic_after:
            logic_interface.close()
        
        return os.path.join(output_dir, timestamp)
    else:
        logic_interface = logic.Logic(sampling_freq=sampling_freq)
        logic_interface.print_saleae_status()

        if capture_format == "both":
            capture_path = logic_interface.export_binary_and_csv_capture(capture_time, output_dir)[0]
        elif capture_format == ".csv":
            capture_path = logic_interface.start_csv_capture(capture_time, output_dir)
        else:
            capture_path = logic_interface.export_binary_capture(capture_time, output_dir)

        if close_logic_after:
            logic_interface.kill_logic()

        return capture_path


def load_hydrophone_data(
        data_path,
        sampling_freq,
        selected_hydrophones,
        is_logic_2,
        plot_data
        ):
    """Load hydrophone data from file.

    Args:
        data_path: Path to data file (.bin or .csv)
        sampling_freq: Sampling frequency in Hz
        selected_hydrophones: List of 4 bools indicating which to load

    Returns:
        HydrophoneArray with loaded data
    """
    array = hydrophone_array.HydrophoneArray(
        sampling_freq=sampling_freq,
        selected=selected_hydrophones
    )
    array.load_from_path(data_path, is_logic_2)
    
    if plot_data:
        array.plot_hydrophones()
    return array


def check_all_valid(toa_results, selected):
    """Check if all selected hydrophones are valid.
    
    Args:
        toa_results: List of TOA analysis results
        selected: List of 4 bools indicating which hydrophones are selected
        
    Returns:
        True only if all selected hydrophones have is_valid=True
    """
    for idx, is_selected in enumerate(selected):
        if is_selected:
            result = next((r for r in toa_results if r['hydrophone_idx'] == idx), None)
            if result is None or not result.get('is_valid', False):
                return False
    return True


def find_closest_hydrophone(analysis_results, selected=None):
    """Find the closest hydrophone based on TOA analysis and nearby status.
    
    Args:
        analysis_results: List of analysis results from run_controller
        selected: List of 4 bools indicating which hydrophones are selected
        
    Returns:
        tuple: (closest_hydrophone_index, is_nearby, all_valid) where 
               all_valid is True only if all selected hydrophones are valid
    """
    if not analysis_results or len(analysis_results) == 0:
        return (None, False, False)
    
    # Get TOA results (assumes first analyzer is TOA-based)
    toa_results = analysis_results[0]['results']
    
    # Find hydrophone with earliest TOA time
    earliest_time = float('inf')
    closest_hydrophone = None
    
    for result in toa_results:
        idx = result['hydrophone_idx']
        toa_time = result.get('toa_time')
        
        if toa_time is not None and toa_time < earliest_time:
            earliest_time = toa_time
            closest_hydrophone = idx
    
    # Get nearby status (assumes second analyzer is nearby analyzer)
    is_nearby = False
    if len(analysis_results) > 1:
        nearby_results = analysis_results[1]['results']
        nearby_count = sum(1 for r in nearby_results if r.get('nearby', False))
        # Consider "nearby" if majority of hydrophones detect it nearby
        is_nearby = nearby_count >= len(nearby_results) / 2
    
    # Check if all selected hydrophones are valid
    all_valid = check_all_valid(toa_results, selected) if selected is not None else True
    
    return (closest_hydrophone, is_nearby, all_valid)


def main():
    # Whether to use Logic 2 or Logic 1
    IS_LOGIC_2 = True

    # Whether to use mock device for Logic 2 (True) or real device (False)
    USE_MOCK_DEVICE = False

    # Whether to capture new data from Logic hardware (True) or use existing file (False)
    CAPTURE_NEW_DATA = True

    # Path to existing data file (used when CAPTURE_NEW_DATA = False)
    DATA_FILE = "data/2.28.2026/0_2026-02-28--14-50-32/0_epoch_0.bin"

    # Duration of capture in seconds (only used if CAPTURE_NEW_DATA = True)
    CAPTURE_TIME = 2

    # Format for capture: '.bin', '.csv', or 'both' (only used if CAPTURE_NEW_DATA = True)
    CAPTURE_FORMAT = ".bin"

    # Output directory for captured data (only used if CAPTURE_NEW_DATA = True)
    CAPTURE_OUTPUT = "Temp_Data"

    # Sampling frequency in Hz for data acquisition
    SAMPLING_FREQ = 781250

    # Which hydrophones to load/analyze (array of 4 booleans)
    SELECTED = [True, True, True, True]

    # Whether to plot raw signal and frequency spectrum
    PLOT_DATA = False

    # Analyzer(s) for TOA detection (set to None to skip analysis)
    ANALYZERS = [
        TOAEnvelopeAnalyzer(
            threshold_sigma=5,
            raw_signal_threshold=1,
            margin_front=0.1,
            margin_end=0.1,
            filter_order=0,
            search_band_min=25000,
            search_band_max=40000,
            plot_results=False
        ),
        NearbyAnalyzer(
            threshold=0.5,
            filter_order=0,
            search_band_min=25000,
            search_band_max=40000,
            plot_results=False
        ),
    ]

    # Step 1: Get data (capture new or load existing)
    if CAPTURE_NEW_DATA:
        timestamp = time.strftime('%Y-%m-%d--%H-%M-%S')
        output_dir = os.path.join(CAPTURE_OUTPUT, timestamp)
        DATA_PATH = capture_data(
            sampling_freq=SAMPLING_FREQ,
            capture_time=CAPTURE_TIME,
            capture_format=CAPTURE_FORMAT,
            output_dir=CAPTURE_OUTPUT,
            is_logic_2=IS_LOGIC_2,
            is_mock=USE_MOCK_DEVICE
        )
    else:
        DATA_PATH = DATA_FILE

    # Step 2: Load data into hydrophone array
    hydrophone_array_obj = load_hydrophone_data(
        data_path=DATA_PATH,
        sampling_freq=SAMPLING_FREQ,
        selected_hydrophones=SELECTED,
        is_logic_2=IS_LOGIC_2,
        plot_data=PLOT_DATA
    )

    # Step 3: Run analysis
    analysis_results = run_controller(
        hydrophone_array=hydrophone_array_obj,
        analyzers=ANALYZERS
    )
    
    # Step 4: Find closest hydrophone and nearby status
    closest, is_nearby, all_valid = find_closest_hydrophone(analysis_results, SELECTED)
    
    print(f"\n{'='*60}")
    print(f"CLOSEST HYDROPHONE: {closest}")
    print(f"IS NEARBY: {is_nearby}")
    print(f"ALL VALID: {all_valid}")
    print(f"{'='*60}")
    
    return (closest, is_nearby, all_valid)

if __name__ == "__main__":
    main()