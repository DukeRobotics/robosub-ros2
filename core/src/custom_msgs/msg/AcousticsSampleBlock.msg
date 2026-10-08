# header.stamp is the acquisition time of the first row. No independent channel clocks.
std_msgs/Header header
string[] channel_ids
float64 sample_rate_hz
string units
uint64 first_sample_index
string timestamp_reference
# Exactly four channels; row-major samples[row * 4 + channel].
uint32 sample_count
float64[] samples
