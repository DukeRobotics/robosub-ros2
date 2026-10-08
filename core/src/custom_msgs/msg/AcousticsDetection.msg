# header.stamp is the common trigger time, in the array coordinate frame.
std_msgs/Header header
uint64 start_sample_index
uint64 end_sample_index
uint64 onset_sample_index
bool has_frequency
float64 frequency_hz
# All four waveforms retain the original time grid. Calibration corrects delays, not sample positions.
# End index is exclusive.
custom_msgs/AcousticsSampleBlock window
string[] quality_names
float64[] quality_values
string status
string reason
