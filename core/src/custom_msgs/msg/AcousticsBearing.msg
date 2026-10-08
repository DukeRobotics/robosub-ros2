# Directions point from the array toward the source in header.frame_id.
# Azimuth = atan2(y, x); elevation = atan2(z, hypot(x, y)); angles in radians.
std_msgs/Header header
string status
string reason
string ambiguity
geometry_msgs/Vector3[] directions
float64[] azimuth_rad
float64[] elevation_rad
float64[] residual_rms_m
custom_msgs/AcousticsTDOA[] pairs
bool has_position
geometry_msgs/Point position
# Empty until a characterized uncertainty model exists; not an implicit zero covariance.
float64[] uncertainty
