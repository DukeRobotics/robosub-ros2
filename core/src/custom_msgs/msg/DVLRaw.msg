# This message contains data sent by a Teledyne Navigator Workhorse
# Water-Mass data is not included

std_msgs/Header header

# System attitude data
float64 sa_roll
float64 sa_pitch
float64 sa_heading
bool sa_valid

# Timing and Scaling Data
float64 ts_salinity
float64 ts_temperature
float64 ts_depth
float64 ts_sound_speed
int32 ts_built_in_test

# Bottom-Track, Instrument-Referenced Velocity Data
float64 bi_x_axis
float64 bi_y_axis
float64 bi_z_axis
float64 bi_error
string bi_status

# Bottom-track quality data. These fields are populated when the DVL provides
# per-beam diagnostics; consumers must check bt_quality_valid before using
# them.
bool bt_quality_valid
float64[4] bt_beam_ranges
uint8[4] bt_correlation
uint8[4] bt_intensity
uint8[4] bt_percent_good
uint8[4] bt_rssi

# Bottom-Track, Ship-Referenced Velocity Data
float64 bs_transverse
float64 bs_longitudinal
float64 bs_normal
string bs_status

# Bottom-Track, Earth-Referenced Velocity Data
float64 be_east
float64 be_north
float64 be_upwards
string  be_status

# Bottom-Track, Earth-Referenced Distance Data
float64 bd_east
float64 bd_north
float64 bd_upwards
float64 bd_range
float64 bd_time
