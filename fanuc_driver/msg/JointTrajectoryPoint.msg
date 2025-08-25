# Single point in a joint trajectory
# Used for streaming trajectory execution

# Sequence number for tracking
int32 sequence

# Joint target data
float64[] positions          # Target joint positions in radians
float64[] velocities         # Target joint velocities in rad/s
float64[] accelerations      # Target joint accelerations in rad/s^2
float64 time_from_start      # Time from trajectory start in seconds