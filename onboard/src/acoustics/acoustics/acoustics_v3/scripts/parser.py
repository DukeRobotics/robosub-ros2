"""Batch processing script for hydrophone data analysis."""
import os
import time
import csv
from controller import run_controller, load_hydrophone_data
from analyzers import TOAEnvelopeAnalyzer, NearbyAnalyzer

if __name__ == "__main__":
    # Configuration
    SAMPLING_FREQ = 781250
    SELECTED = [True, False, True, False]
    PLOT_DATA = False
    
    DATA_PATHS = [
        "data/2.15.2026/0_logic1_hydrophone_0_closest_2026-02-15--15-52-54",
        "data/2.15.2026/2_logic1_hydrophone_2_closest_2026-02-15--15-55-42"
    ]
    
    ANALYZERS = [
        TOAEnvelopeAnalyzer(
            threshold_sigma=5,
            filter_order=16,
            search_band_min=25000,
            search_band_max=40000,
            plot_results=False
        ),
        NearbyAnalyzer(
            threshold=1.0,
            filter_order=0,
            search_band_min=25000,
            search_band_max=40000,
            plot_results=False
        ),
    ]
    
    # Setup output CSV
    timestamp = time.strftime('%Y-%m-%d--%H-%M-%S')
    OUTPUT_PATH = os.path.join("analysis", f"analysis_{timestamp}.csv")
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    
    HEADERS = ["PATH", "TRUTH", "PREDICTED",
               "H0 TOA", "H1 TOA", "H2 TOA", "H3 TOA",
               "H0 NEARBY", "H1 NEARBY", "H2 NEARBY", "H3 NEARBY"]
    
    with open(OUTPUT_PATH, mode="w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(HEADERS)
    
    # Initialize tracking
    confusion = {i: {j: 0 for j in range(4)} for i in range(4)}
    total_files = 0
    errors = 0
    
    # Process all data files
    for data_dir in DATA_PATHS:
        dir_name = os.path.basename(os.path.normpath(data_dir))
        try:
            truth = int(dir_name.split("_")[0])
        except (ValueError, IndexError):
            truth = None
        
        for filename in os.listdir(data_dir):
            if not filename.endswith('.bin'):
                continue
            
            filepath = os.path.join(data_dir, filename)
            total_files += 1
            
            try:
                # Load and analyze
                array = load_hydrophone_data(
                    data_path=filepath,
                    sampling_freq=SAMPLING_FREQ,
                    selected_hydrophones=SELECTED,
                    plot_data=PLOT_DATA
                )
                
                results = run_controller(
                    hydrophone_array=array,
                    analyzers=ANALYZERS
                )
                
                # Find closest hydrophone by earliest TOA time
                toa_results = results[0]['results']
                latest_time = 0
                predicted = None
                
                for result in toa_results:
                    idx = result['hydrophone_idx']
                    toa_time = result.get('toa_time')
                    
                    if toa_time is not None and toa_time > latest_time:
                        latest_time = toa_time
                        predicted = idx
                
                # Extract TOA times and nearby status for CSV
                toa_dict = {r['hydrophone_idx']: r['toa_time'] for r in toa_results}
                nearby_dict = {r['hydrophone_idx']: r['nearby'] for r in results[1]['results']}
                
                toas = [toa_dict.get(i) for i in range(4)]
                nearby_status = [nearby_dict.get(i) for i in range(4)]
                
                # Write to CSV
                row = [filename, truth, predicted] + toas + nearby_status
                with open(OUTPUT_PATH, mode="a", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(row)
                
                # Update confusion matrix
                if truth is not None and predicted is not None:
                    confusion[truth][predicted] += 1
                
                print(f"Processed: {filename} | Predicted: H{predicted} | Truth: H{truth}")
                
            except Exception as e:
                print(f"Error: {filepath} - {e}")
                errors += 1
    
    # Print metrics
    correct = sum(confusion[i][i] for i in range(4))
    accuracy = 100 * correct / total_files if total_files > 0 else 0
    
    print(f"\n{'='*60}")
    print("ACCURACY METRICS")
    print(f"{'='*60}")
    print(f"Total files: {total_files}")
    print(f"Errors: {errors}")
    print(f"Accuracy: {correct}/{total_files} ({accuracy:.1f}%)")
    
    print("\nConfusion Matrix:")
    print("       Predicted")
    print("       0   1   2   3")
    print("     " + "-" * 17)
    for i in range(4):
        counts = [confusion[i][j] for j in range(4)]
        if sum(counts) > 0:
            print(f"True {i} | {counts[0]:2d}  {counts[1]:2d}  {counts[2]:2d}  {counts[3]:2d}")
    
    print(f"\n{'='*60}")
    print(f"Results: {OUTPUT_PATH}")
    print(f"{'='*60}")