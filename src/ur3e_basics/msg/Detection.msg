# Header with timestamp and frame
std_msgs/Header header

# Object identification
string object_id
string object_class
float32 confidence

# 2D bounding box
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# 3D pose in a camera frame
geometry_msgs/PoseStamped pose