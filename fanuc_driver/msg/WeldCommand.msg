# Fanuc Welding Command Message
# This message corresponds to the ind_weld_cmd_data_t structure in the KAREL code
# and represents welding control commands to be sent to the Fanuc robot controller
# for execution via Group Outputs (GO) and Digital Outputs (DO).

# Group Output (GO) commands - analog/numeric values
int32 target_wire_spd   # GO[3] Target wire feed speed (0.1 m/min units)
int32 correction_val    # GO[4] Correction value (-999 to +999)
int32 dyn_setting      # GO[5] Dynamic setting (0-100)
int32 operation_mode   # GO[6] Operation mode (0=Standard, 1=Pulse, etc.)
int32 std_pulse_val    # GO[7] Standard/Pulse value (depends on mode)
int32 program_number   # GO[2] Welding program number (1-99)

# Digital Output (DO) commands - binary control signals
# Use 0 for OFF, any non-zero value for ON
int32 arc_start_cmd    # DO[253] Arc start command
int32 gas_control      # DO[255] Gas control (shielding gas)
int32 jog_feed_cmd     # DO[257] Jog wire feed (manual wire feed)
int32 jog_retract_cmd  # DO[259] Jog wire retract (manual wire retract)

# Header information for tracking
std_msgs/Header header 