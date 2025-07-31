# Fanuc Welding State Message
# This message corresponds to the weld_state_t structure in the KAREL code
# and represents the real-time welding status from an EWM welding power source
# connected via fieldbus (Profinet/EthernetIP) to the Fanuc robot controller.

# Boolean status flags (extracted from EWM status word)
bool arc_ok         # I>0 signal - Arc is established and stable
bool ready          # Welder is ready for operation
bool stick_err      # Wire stick error detected
bool power_err      # Power error condition active

# Deposition detection status (original boolean value from DI[251])
bool depos_di       # Deposition detection status from DI[251]

# Analog process values (raw 16-bit integers from EWM)
# These should be scaled on the ROS side according to EWM manual:
# - act_voltage: multiply by 100.0/32767.0 to get volts
# - act_current: multiply by 1000.0/32767.0 to get amperes  
# - act_wire_spd: multiply by 40.0/32767.0 to get m/min
int16 act_voltage   # Raw actual welding voltage
int16 act_current   # Raw actual welding current
int16 act_wire_spd  # Raw actual wire feed speed 