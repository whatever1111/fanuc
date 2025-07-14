#!/usr/bin/env python3

"""
Example script demonstrating how to monitor welding state from a Fanuc robot
connected to an EWM welding power source via fieldbus.

This script subscribes to the /weld_state topic and prints scaled welding
parameters in a human-readable format.
"""

import rospy
from fanuc_driver.msg import WeldState

# Scaling factors from EWM manual (matching the C++ node)
VOLTAGE_SCALE = 100.0 / 32767.0   # Raw to Volts
CURRENT_SCALE = 1000.0 / 32767.0  # Raw to Amperes
WIRE_SPEED_SCALE = 40.0 / 32767.0 # Raw to m/min

class WeldStateMonitor:
    def __init__(self):
        rospy.init_node('weld_state_monitor')
        
        # Subscribe to welding state
        self.weld_sub = rospy.Subscriber('/weld_state', WeldState, self.weld_state_callback)
        
        rospy.loginfo("Welding State Monitor started. Listening for welding data...")
        
    def weld_state_callback(self, msg):
        """
        Callback function to process incoming welding state messages.
        
        Args:
            msg (WeldState): The welding state message
        """
        # Scale the raw values according to EWM specifications
        voltage = msg.act_voltage * VOLTAGE_SCALE
        current = msg.act_current * CURRENT_SCALE
        wire_speed = msg.act_wire_spd * WIRE_SPEED_SCALE
        
        # Create status string
        arc_status = "ARC ON" if msg.arc_ok else "ARC OFF"
        ready_status = "READY" if msg.ready else "NOT READY"
        
        # Print formatted output
        rospy.loginfo(f"Weld Status: {arc_status} | {ready_status} | "
                     f"V: {voltage:.1f}V | I: {current:.0f}A | "
                     f"Wire: {wire_speed:.1f}m/min")
        
        # Print warnings for error conditions
        if msg.stick_err:
            rospy.logwarn("Wire stick error detected!")
            
        if msg.general_err:
            rospy.logwarn(f"General welding error - Code: {msg.err_code}")
            
    def run(self):
        """Main execution loop."""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Welding State Monitor shutting down...")

if __name__ == '__main__':
    try:
        monitor = WeldStateMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass 