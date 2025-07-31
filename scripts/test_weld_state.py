#!/usr/bin/env python

"""
Test script for Fanuc welding state monitoring.
This script subscribes to the welding state topic and displays the received data.
"""

import rospy
from fanuc_driver.msg import WeldState
import signal
import sys

class WeldStateMonitor:
    def __init__(self):
        rospy.init_node('weld_state_monitor', anonymous=True)
        
        # Subscribe to welding state topic
        self.weld_sub = rospy.Subscriber('/fanuc_weld_state/weld_state', WeldState, self.weld_callback)
        
        # Statistics
        self.msg_count = 0
        self.last_msg_time = rospy.Time.now()
        
        # Scaling factors (same as in C++ node)
        self.VOLTAGE_SCALE = 100.0 / 32767.0
        self.CURRENT_SCALE = 1000.0 / 32767.0
        self.WIRE_SPEED_SCALE = 40.0 / 32767.0
        
        rospy.loginfo("=== Welding State Monitor Started ===")
        rospy.loginfo("Waiting for welding state messages...")
        rospy.loginfo("Press Ctrl+C to stop")
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def weld_callback(self, msg):
        """Callback for welding state messages"""
        self.msg_count += 1
        current_time = rospy.Time.now()
        
        # Calculate message frequency
        time_diff = (current_time - self.last_msg_time).to_sec()
        freq = 1.0 / time_diff if time_diff > 0 else 0
        self.last_msg_time = current_time
        
        # Calculate scaled values
        voltage_scaled = msg.act_voltage * self.VOLTAGE_SCALE
        current_scaled = msg.act_current * self.CURRENT_SCALE
        wire_speed_scaled = msg.act_wire_spd * self.WIRE_SPEED_SCALE
        
        # Display message information
        rospy.loginfo("=== Message #%d (%.1f Hz) ===", self.msg_count, freq)
        rospy.loginfo("Timestamp: %s", msg.header.stamp)
        rospy.loginfo("Boolean States:")
        rospy.loginfo("  Arc OK: %s", "YES" if msg.arc_ok else "NO")
        rospy.loginfo("  Ready: %s", "YES" if msg.ready else "NO")
        rospy.loginfo("  Stick Error: %s", "YES" if msg.stick_err else "NO")
        rospy.loginfo("  General Error: %s", "YES" if msg.general_err else "NO")
        rospy.loginfo("Error Code: %d", msg.err_code)
        rospy.loginfo("Raw Values:")
        rospy.loginfo("  Voltage: %d (%.2f V)", msg.act_voltage, voltage_scaled)
        rospy.loginfo("  Current: %d (%.1f A)", msg.act_current, current_scaled)
        rospy.loginfo("  Wire Speed: %d (%.2f m/min)", msg.act_wire_spd, wire_speed_scaled)
        rospy.loginfo("")
        
        # Check for potential issues
        if msg.general_err:
            rospy.logwarn("General error detected! Error code: %d", msg.err_code)
        if msg.stick_err:
            rospy.logwarn("Wire stick error detected!")
        if not msg.ready:
            rospy.loginfo("Welder not ready")
        if not msg.arc_ok:
            rospy.loginfo("Arc not stable")
            
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        rospy.loginfo("")
        rospy.loginfo("=== Test Summary ===")
        rospy.loginfo("Total messages received: %d", self.msg_count)
        rospy.loginfo("Shutting down...")
        sys.exit(0)
        
    def run(self):
        """Main loop"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        monitor = WeldStateMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr("Error in test script: %s", str(e)) 