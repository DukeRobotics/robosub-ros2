std_msgs/Header header

# Max/min x- and y-coordinates for the bounding box
float64 xmin
float64 ymin
float64 xmax
float64 ymax

# Yaw value for task planning
float64 yaw

# Object coordinates
geometry_msgs/Point coords

int32 height
int32 width

float64 distance

# Sonar scan parameters
float64 sonar_start_angle  # Start angle for sonar sweep (degrees)
float64 sonar_end_angle    # End angle for sonar sweep (degrees)
float64 sonar_scan_distance # Distance for sonar scan (meters)

string label
float64 score
bool sonar # True = using sonar
