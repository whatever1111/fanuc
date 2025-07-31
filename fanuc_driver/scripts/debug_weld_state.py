#!/usr/bin/env python3

"""
Debug script for Fanuc Welding State monitoring.
This script helps verify the I/O mapping and parsing logic corrections.
"""

import rospy
from fanuc_driver.msg import WeldState

# Scaling factors from EWM manual (matching the C++ nodes)
VOLTAGE_SCALE = 100.0 / 32767.0   # Raw to Volts
CURRENT_SCALE = 1000.0 / 32767.0  # Raw to Amperes
WIRE_SPEED_SCALE = 40.0 / 32767.0 # Raw to m/min

class WeldStateDebugger:
    def __init__(self):
        rospy.init_node('weld_state_debugger')
        
        # Subscribe to welding state
        self.weld_sub = rospy.Subscriber('/weld_state', WeldState, self.weld_state_callback)
        
        self.msg_count = 0
        
        rospy.loginfo("=== Weld State Debugger Started ===")
        rospy.loginfo("I/O Mapping Reference:")
        rospy.loginfo("  DI[249] -> arc_ok (arc detection)")
        rospy.loginfo("  DI[252] -> power_err (power error)")
        rospy.loginfo("  DI[251] -> depos_di (deposition detection - original boolean value)")
        rospy.loginfo("  GI[2]   -> act_voltage (raw voltage)")
        rospy.loginfo("  GI[3]   -> act_current (raw current)")
        rospy.loginfo("")
        
    def weld_state_callback(self, msg):
        """
        Callback function to process and debug welding state messages.
        """
        self.msg_count += 1
        
        # Scale the raw values
        voltage = msg.act_voltage * VOLTAGE_SCALE
        current = msg.act_current * CURRENT_SCALE
        wire_speed = msg.act_wire_spd * WIRE_SPEED_SCALE
        
        rospy.loginfo("=== Message #%d ===", self.msg_count)
        rospy.loginfo("Digital Inputs:")
        rospy.loginfo("  DI[249] arc_ok:      %s", "TRUE" if msg.arc_ok else "FALSE")
        rospy.loginfo("  DI[252] power_err:   %s", "TRUE" if msg.power_err else "FALSE")
        rospy.loginfo("  DI[251] depos_di:    %s (original value)", "TRUE" if msg.depos_di else "FALSE")
        
        rospy.loginfo("Analog Inputs:")
        rospy.loginfo("  GI[2] voltage: %d (%.1fV)", msg.act_voltage, voltage)
        rospy.loginfo("  GI[3] current: %d (%.0fA)", msg.act_current, current)
        
        rospy.loginfo("Status Summary:")
        rospy.loginfo("  Ready:           %s (Fixed for testing)", "TRUE" if msg.ready else "FALSE")
        rospy.loginfo("  Stick Error:     %s (Fixed for testing)", "TRUE" if msg.stick_err else "FALSE")
        rospy.loginfo("  Wire Speed:      %d (%.1fm/min, Fixed)", msg.act_wire_spd, wire_speed)
        
        # Validation checks
        if msg.arc_ok and voltage > 0 and current > 0:
            rospy.loginfo("✓ Arc status and power values are consistent")
        elif not msg.arc_ok and voltage == 0 and current == 0:
            rospy.loginfo("✓ No arc and zero power values are consistent")
        else:
            rospy.logwarn("⚠ Arc status and power values may be inconsistent")
            
        if msg.power_err:
            rospy.logwarn("⚠ Power error active (DI[252])")
            
        if not msg.depos_di:
            rospy.logwarn("⚠ No deposition detected (DI[251])")
        else:
            rospy.loginfo("✓ Deposition detected (DI[251])")
            
        rospy.loginfo("")

if __name__ == '__main__':
    try:
        debugger = WeldStateDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Weld State Debugger stopped") 