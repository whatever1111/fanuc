#!/usr/bin/env python3
"""
Welding State Message Validator
Monitors /weld_state topic and validates message content and timing
"""

import rospy
from fanuc_driver.msg import WeldState
import time
import sys
from collections import deque
import statistics

class WeldStateValidator:
    def __init__(self):
        self.last_msg_time = None
        self.msg_count = 0
        self.error_count = 0
        self.warning_count = 0
        
        # Statistics tracking
        self.message_intervals = deque(maxlen=100)  # Keep last 100 intervals
        self.voltage_values = deque(maxlen=50)
        self.current_values = deque(maxlen=50)
        self.wire_speed_values = deque(maxlen=50)
        
        # Error tracking
        self.consecutive_errors = 0
        self.max_consecutive_errors = 0
        
        # Timing
        self.start_time = time.time()
        
    def validate_message(self, msg):
        """Validate incoming welding state message"""
        current_time = time.time()
        self.msg_count += 1
        
        # Check message timing
        if self.last_msg_time:
            dt = current_time - self.last_msg_time
            self.message_intervals.append(dt)
            
            if dt > 0.5:  # Expect messages more frequently than every 500ms
                self.log_warning(f"Large message gap: {dt:.3f}s")
            elif dt < 0.01:  # Messages coming too fast
                self.log_warning(f"Messages too frequent: {dt:.3f}s")
        
        self.last_msg_time = current_time
        
        # Validate message content
        errors_found = False
        
        # Check error code range
        if msg.err_code > 255:
            self.log_error(f"Invalid error code: {msg.err_code} (should be 0-255)")
            errors_found = True
            
        # Check for reasonable analog values (16-bit signed range)
        if abs(msg.act_voltage) > 32767:
            self.log_error(f"Voltage out of 16-bit range: {msg.act_voltage}")
            errors_found = True
            
        if abs(msg.act_current) > 32767:
            self.log_error(f"Current out of 16-bit range: {msg.act_current}")
            errors_found = True
            
        if abs(msg.act_wire_spd) > 32767:
            self.log_error(f"Wire speed out of 16-bit range: {msg.act_wire_spd}")
            errors_found = True
        
        # Check scaled values for reasonableness
        voltage_scaled = msg.act_voltage * 100.0 / 32767.0
        current_scaled = msg.act_current * 1000.0 / 32767.0
        wire_speed_scaled = msg.act_wire_spd * 40.0 / 32767.0
        
        # Store for statistics
        self.voltage_values.append(voltage_scaled)
        self.current_values.append(current_scaled)
        self.wire_speed_values.append(wire_speed_scaled)
        
        # Check for typical welding ranges
        if msg.arc_ok:
            if not (5.0 <= voltage_scaled <= 50.0):
                self.log_warning(f"Voltage unusual for welding: {voltage_scaled:.1f}V")
            
            if not (20.0 <= current_scaled <= 600.0):
                self.log_warning(f"Current unusual for welding: {current_scaled:.0f}A")
            
            if not (1.0 <= wire_speed_scaled <= 25.0):
                self.log_warning(f"Wire speed unusual: {wire_speed_scaled:.1f}m/min")
        
        # Check for stuck values (all zeros or same value repeated)
        if len(self.voltage_values) >= 10:
            if len(set(list(self.voltage_values)[-10:])) == 1:
                self.log_warning("Voltage appears stuck (same value for 10 messages)")
        
        # Track consecutive errors
        if errors_found:
            self.consecutive_errors += 1
            self.max_consecutive_errors = max(self.max_consecutive_errors, self.consecutive_errors)
        else:
            self.consecutive_errors = 0
        
        # Print periodic statistics
        if self.msg_count % 50 == 0:
            self.print_statistics()
            
    def log_error(self, message):
        """Log error message"""
        rospy.logerr(f"[MSG {self.msg_count}] ERROR: {message}")
        self.error_count += 1
        
    def log_warning(self, message):
        """Log warning message"""
        rospy.logwarn(f"[MSG {self.msg_count}] WARNING: {message}")
        self.warning_count += 1
        
    def print_statistics(self):
        """Print periodic statistics"""
        runtime = time.time() - self.start_time
        msg_rate = self.msg_count / runtime if runtime > 0 else 0
        error_rate = self.error_count / self.msg_count * 100 if self.msg_count > 0 else 0
        warning_rate = self.warning_count / self.msg_count * 100 if self.msg_count > 0 else 0
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"VALIDATION STATISTICS (Runtime: {runtime:.1f}s)")
        rospy.loginfo(f"Messages: {self.msg_count} ({msg_rate:.1f} Hz)")
        rospy.loginfo(f"Errors: {self.error_count} ({error_rate:.1f}%)")
        rospy.loginfo(f"Warnings: {self.warning_count} ({warning_rate:.1f}%)")
        rospy.loginfo(f"Max consecutive errors: {self.max_consecutive_errors}")
        
        # Message timing statistics
        if len(self.message_intervals) > 1:
            avg_interval = statistics.mean(self.message_intervals)
            min_interval = min(self.message_intervals)
            max_interval = max(self.message_intervals)
            rospy.loginfo(f"Message intervals: avg={avg_interval:.3f}s, min={min_interval:.3f}s, max={max_interval:.3f}s")
        
        # Value statistics
        if len(self.voltage_values) > 1:
            avg_voltage = statistics.mean(self.voltage_values)
            min_voltage = min(self.voltage_values)
            max_voltage = max(self.voltage_values)
            rospy.loginfo(f"Voltage range: {min_voltage:.1f}V to {max_voltage:.1f}V (avg: {avg_voltage:.1f}V)")
        
        if len(self.current_values) > 1:
            avg_current = statistics.mean(self.current_values)
            min_current = min(self.current_values)
            max_current = max(self.current_values)
            rospy.loginfo(f"Current range: {min_current:.0f}A to {max_current:.0f}A (avg: {avg_current:.0f}A)")
        
        rospy.loginfo("=" * 60)
        
    def print_final_report(self):
        """Print final validation report"""
        runtime = time.time() - self.start_time
        
        print("\n" + "=" * 60)
        print("FINAL VALIDATION REPORT")
        print("=" * 60)
        print(f"Total runtime: {runtime:.1f} seconds")
        print(f"Total messages: {self.msg_count}")
        print(f"Message rate: {self.msg_count / runtime:.1f} Hz")
        print(f"Total errors: {self.error_count}")
        print(f"Total warnings: {self.warning_count}")
        print(f"Error rate: {self.error_count / self.msg_count * 100:.1f}%")
        print(f"Warning rate: {self.warning_count / self.msg_count * 100:.1f}%")
        
        # Overall assessment
        if self.error_count == 0 and self.warning_count < self.msg_count * 0.05:
            print("\n✓ VALIDATION PASSED - System appears to be working correctly")
        elif self.error_count == 0:
            print(f"\n⚠ VALIDATION PASSED WITH WARNINGS - {self.warning_count} warnings detected")
        else:
            print(f"\n✗ VALIDATION FAILED - {self.error_count} errors detected")
        
        print("=" * 60)

def main():
    rospy.init_node('weld_state_validator', log_level=rospy.INFO)
    
    validator = WeldStateValidator()
    
    rospy.loginfo("Welding State Validator started")
    rospy.loginfo("Monitoring /weld_state topic for message validation...")
    rospy.loginfo("Press Ctrl+C to stop and see final report")
    
    def callback(msg):
        validator.validate_message(msg)
    
    try:
        sub = rospy.Subscriber('/weld_state', WeldState, callback)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Validation stopped by user")
    finally:
        validator.print_final_report()

if __name__ == '__main__':
    main() 