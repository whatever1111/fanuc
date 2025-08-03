# Fanuc Welding State Message
# This message corresponds to the weld_state_t structure in the KAREL code
# and represents the real-time welding status from an EWM welding power source
# connected via fieldbus (Profinet/EthernetIP) to the Fanuc robot controller.

# Boolean status flags (extracted from EWM status word and digital inputs)
bool arc_ok         # I>0 signal - Arc is established and stable from DI[249]
bool power_err      # Power error condition active from DI[252]
bool depos_di       # Deposition detection status from DI[251]

# Raw status words from EWM (16-bit integers, see manual 099-008225-EWM21)
int16 ewm_err       # EWM error status word (GI[1])
int16 warning_state # Warning status word  (GI[6])

# Analog process values (raw 16-bit integers from EWM)
# - act_voltage  : multiply by 100.0/32767.0 to get volts
# - act_current  : multiply by 1000.0/32767.0 to get amperes
# - act_wire_spd : multiply by 40.0/32767.0 to get m/min
# - motor_current: multiply by 1000.0/32767.0 to get amperes
int16 act_voltage   # Raw actual welding voltage  (GI[2])
int16 act_current   # Raw actual welding current  (GI[3])
int16 act_wire_spd  # Raw actual wire feed speed  (GI[4])
int16 motor_current # Raw motor current           (GI[5])
