# Industrial robot status message
# Modeled after ROS-Industrial simple_message STATUS

# Robot Mode Constants
int8 ROBOT_MODE_UNKNOWN=-1
int8 ROBOT_MODE_MANUAL=1
int8 ROBOT_MODE_AUTO=2

# Tri-State Constants  
int8 TRI_STATE_UNKNOWN=-1
int8 TRI_STATE_FALSE=0
int8 TRI_STATE_TRUE=1

# E-Stop Constants
int8 E_STOP_UNKNOWN=-1
int8 E_STOP_ARMED=0
int8 E_STOP_TRIGGERED=1

# Error Code Constants
int32 ERROR_CODE_NONE=0

# Fields
int8 mode                    # Robot mode (manual/auto)
int8 e_stopped               # E-stop state
int8 drives_powered          # Servo power status  
int8 motion_possible         # Ready for motion
int8 in_motion               # Currently executing motion
int8 in_error                # Error state
int32 error_code             # Specific error code if in_error

std_msgs/Header header       # Standard ROS2 header with timestamp