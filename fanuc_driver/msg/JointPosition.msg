# Joint position feedback message
# Contains current joint positions from robot controller

# Maximum number of joints supported
int32 MAX_JOINTS=10

# Sequence number for tracking messages
int32 sequence

# Joint position data
float64[] positions          # Joint positions in radians
float64[] velocities         # Joint velocities in rad/s (optional, may be empty)
float64[] accelerations      # Joint accelerations in rad/s^2 (optional, may be empty)

std_msgs/Header header       # Standard ROS2 header with timestamp