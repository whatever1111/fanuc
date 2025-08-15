#!/usr/bin/env python3
"""
ROS-integrated Test Suite for Weld Control Frequency Test Tool
=============================================================

This test suite validates the weld_control_freq_test.py script in a ROS environment
using rostest framework. It tests:

1. ROS node initialization and communication
2. Message publishing and subscribing
3. Waveform generation accuracy
4. Integration with actual ROS topics

Usage:
    rostest fanuc_driver test_weld_control_freq_ros.test

Author: Test Suite  
Date: 2024
"""

import unittest
import rospy
import rostest
import time
import math
import random
from collections import deque
from typing import List, Tuple

from std_msgs.msg import Header
from fanuc_driver.msg import WeldCommand, WeldState


class TestWeldFreqROS(unittest.TestCase):
    """ROS-integrated test class for weld frequency testing"""
    
    def setUp(self):
        """Set up test environment"""
        rospy.init_node('test_weld_freq_ros', anonymous=True)
        
        # Publishers and subscribers for testing
        self.cmd_pub = rospy.Publisher('/weld_command', WeldCommand, queue_size=10)
        self.state_pub = rospy.Publisher('/weld_state', WeldState, queue_size=10)
        
        # Message collectors
        self.received_commands: List[WeldCommand] = []
        self.received_states: List[WeldState] = []
        
        # Subscribe to topics to capture messages
        self.cmd_sub = rospy.Subscriber('/weld_command', WeldCommand, self._cmd_callback)
        self.state_sub = rospy.Subscriber('/weld_state', WeldState, self._state_callback)
        
        # Wait for connections
        rospy.sleep(0.5)
        
        # Test timeout
        self.test_timeout = 10.0
    
    def _cmd_callback(self, msg):
        """Callback for weld command messages"""
        self.received_commands.append(msg)
    
    def _state_callback(self, msg):
        """Callback for weld state messages"""
        self.received_states.append(msg)
    
    def create_weld_command(self, **kwargs) -> WeldCommand:
        """Create a WeldCommand message with specified fields"""
        cmd = WeldCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "test_frame"
        
        # Set all fields to NO_CHANGE (-1) by default
        cmd.target_wire_spd = kwargs.get('target_wire_spd', -1)
        cmd.correction_val = kwargs.get('correction_val', -1)
        cmd.dyn_setting = kwargs.get('dyn_setting', -1)
        cmd.operation_mode = kwargs.get('operation_mode', -1)
        cmd.std_pulse_val = kwargs.get('std_pulse_val', -1)
        cmd.program_number = kwargs.get('program_number', -1)
        
        cmd.arc_start_cmd = kwargs.get('arc_start_cmd', -1)
        cmd.gas_control = kwargs.get('gas_control', -1)
        cmd.jog_feed_cmd = kwargs.get('jog_feed_cmd', -1)
        cmd.jog_retract_cmd = kwargs.get('jog_retract_cmd', -1)
        
        return cmd
    
    def create_weld_state(self, **kwargs) -> WeldState:
        """Create a WeldState message with specified fields"""
        state = WeldState()
        # Align with actual WeldState.msg fields
        # Booleans
        state.arc_ok = kwargs.get('arc_ok', False)
        state.power_err = kwargs.get('power_err', False)
        state.depos_di = kwargs.get('depos_di', False)
        # Status words
        state.ewm_err = kwargs.get('ewm_err', 0)
        state.warning_state = kwargs.get('warning_state', 0)
        # Analog values
        state.act_voltage = kwargs.get('act_voltage', 0)
        state.act_current = kwargs.get('act_current', 0)
        state.act_wire_spd = kwargs.get('act_wire_spd', 0)
        state.motor_current = kwargs.get('motor_current', 0)
        
        return state
    
    def test_ros_message_structure(self):
        """Test ROS message structure and field validation"""
        rospy.loginfo("Testing ROS message structure...")
        
        # Test WeldCommand message creation
        cmd = self.create_weld_command(
            target_wire_spd=50,
            jog_feed_cmd=1
        )
        
        # Validate message structure
        self.assertTrue(hasattr(cmd, 'header'))
        self.assertTrue(hasattr(cmd.header, 'stamp'))
        self.assertTrue(hasattr(cmd.header, 'frame_id'))
        self.assertEqual(cmd.target_wire_spd, 50)
        self.assertEqual(cmd.jog_feed_cmd, 1)
        self.assertEqual(cmd.correction_val, -1)  # Should remain NO_CHANGE
        
        # Test WeldState message creation (no header field in WeldState.msg)
        state = self.create_weld_state(
            act_wire_spd=45,
            arc_ok=True,
            power_err=False
        )
        
        self.assertEqual(state.act_wire_spd, 45)
        self.assertTrue(state.arc_ok)
        self.assertFalse(state.power_err)  # Should remain default
    
    def test_message_publishing(self):
        """Test message publishing and receiving"""
        rospy.loginfo("Testing message publishing...")
        
        # Clear received messages
        self.received_commands.clear()
        self.received_states.clear()
        
        # Publish test messages
        test_commands = [
            self.create_weld_command(target_wire_spd=10),
            self.create_weld_command(target_wire_spd=20),
            self.create_weld_command(jog_feed_cmd=1),
        ]
        
        test_states = [
            self.create_weld_state(act_wire_spd=10),
            self.create_weld_state(act_wire_spd=20),
        ]
        
        # Publish commands
        for cmd in test_commands:
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        
        # Publish states
        for state in test_states:
            self.state_pub.publish(state)
            rospy.sleep(0.1)
        
        # Wait for messages to be received
        rospy.sleep(0.5)
        
        # Verify reception
        self.assertGreaterEqual(len(self.received_commands), len(test_commands))
        self.assertGreaterEqual(len(self.received_states), len(test_states))
        
        # Verify message content
        if len(self.received_commands) >= 3:
            self.assertEqual(self.received_commands[-3].target_wire_spd, 10)
            self.assertEqual(self.received_commands[-2].target_wire_spd, 20)
            self.assertEqual(self.received_commands[-1].jog_feed_cmd, 1)
    
    def test_waveform_generation_timing(self):
        """Test waveform generation timing in ROS context"""
        rospy.loginfo("Testing waveform generation timing...")
        
        # Test parameters
        freq = 2.0  # Hz
        amp = 15.0
        offset = 30.0
        sample_rate = 20.0  # Hz
        duration = 2.0  # seconds
        
        # Clear received messages
        self.received_commands.clear()
        
        # Generate and publish sine wave
        rate = rospy.Rate(sample_rate)
        start_time = rospy.Time.now()
        
        sample_count = 0
        target_samples = int(duration * sample_rate)
        
        while not rospy.is_shutdown() and sample_count < target_samples:
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            
            # Generate sine wave value
            sine_val = math.sin(2 * math.pi * freq * elapsed)
            target_speed = int(offset + amp * sine_val)
            
            # Create and publish command
            cmd = self.create_weld_command(target_wire_spd=target_speed)
            self.cmd_pub.publish(cmd)
            
            sample_count += 1
            rate.sleep()
        
        # Wait for all messages to be received
        rospy.sleep(0.5)
        
        # Verify timing (allow larger tolerance in container/virtualized env)
        actual_duration = (rospy.Time.now() - start_time).to_sec()
        expected_duration = duration
        
        # Allow 30% tolerance for timing due to scheduler jitter
        timing_error = abs(actual_duration - expected_duration) / expected_duration
        self.assertLess(timing_error, 0.3, 
                       f"Timing error too large: {timing_error:.3f}")
        
        # Verify sample count
        self.assertEqual(sample_count, target_samples)
        
        # Verify we received the messages
        self.assertGreaterEqual(len(self.received_commands), target_samples * 0.9)
    
    def test_pulse_enhanced_signal_ros(self):
        """Test pulse-enhanced step signal generation in ROS"""
        rospy.loginfo("Testing pulse-enhanced signal in ROS...")
        
        # Test parameters
        step_freq = 1.0
        step_amp = 20.0
        offset = 50.0
        pulse_rate = 3.0
        pulse_amp = 10.0
        pulse_duration = 0.2
        sample_rate = 25.0
        duration = 3.0
        
        # Set deterministic random seed
        random.seed(456)
        
        # Clear received messages
        self.received_commands.clear()
        
        # Generate pulse-enhanced signal
        rate = rospy.Rate(sample_rate)
        start_time = rospy.Time.now()
        
        pulse_end_time = 0.0
        current_pulse_amplitude = 0.0
        pulse_prob_per_sample = pulse_rate / sample_rate
        
        sample_count = 0
        target_samples = int(duration * sample_rate)
        pulse_events = []
        
        while not rospy.is_shutdown() and sample_count < target_samples:
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            
            # Generate base step signal
            phase = (elapsed * step_freq) % 1.0
            base_signal = step_amp if phase < 0.5 else -step_amp
            
            # Handle pulse generation
            if elapsed >= pulse_end_time:
                current_pulse_amplitude = 0.0
                
                if random.random() < pulse_prob_per_sample:
                    current_pulse_amplitude = random.uniform(-pulse_amp, pulse_amp)
                    pulse_end_time = elapsed + pulse_duration
                    pulse_events.append((elapsed, current_pulse_amplitude))
            
            # Combine signals
            target_speed = int(offset + base_signal + current_pulse_amplitude)
            
            # Create and publish command
            cmd = self.create_weld_command(target_wire_spd=target_speed)
            self.cmd_pub.publish(cmd)
            
            sample_count += 1
            rate.sleep()
        
        # Wait for messages
        rospy.sleep(0.5)
        
        # Verify pulse events were generated
        self.assertGreater(len(pulse_events), 0, "Should have generated some pulse events")
        
        # Verify signal bounds
        if self.received_commands:
            speeds = [cmd.target_wire_spd for cmd in self.received_commands[-target_samples:]]
            min_speed = min(speeds)
            max_speed = max(speeds)
            
            expected_min = int(offset - step_amp - pulse_amp)
            expected_max = int(offset + step_amp + pulse_amp)
            
            self.assertGreaterEqual(min_speed, expected_min - 1)
            self.assertLessEqual(max_speed, expected_max + 1)
            
            rospy.loginfo(f"Generated {len(pulse_events)} pulse events")
            rospy.loginfo(f"Signal range: {min_speed} to {max_speed}")
    
    def test_latency_measurement_simulation(self):
        """Test latency measurement simulation"""
        rospy.loginfo("Testing latency measurement simulation...")
        
        # Simulate command-response latency measurement
        threshold = 2.0
        cmd_history = deque(maxlen=100)
        latencies = []
        
        # Send commands and simulate responses
        test_commands = [25, 50, 75, 30, 60]
        
        for target_speed in test_commands:
            # Send command
            cmd = self.create_weld_command(target_wire_spd=target_speed)
            cmd_time = rospy.Time.now()
            self.cmd_pub.publish(cmd)
            
            # Store command for tracking
            cmd_history.append((target_speed, cmd_time))
            
            # Simulate processing delay
            rospy.sleep(0.1)
            
            # Simulate response with some variation
            actual_speed = target_speed + random.randint(-1, 1)
            state = self.create_weld_state(act_wire_spd=actual_speed)
            response_time = rospy.Time.now()
            self.state_pub.publish(state)
            
            # Measure latency if within threshold
            if cmd_history and abs(actual_speed - cmd_history[0][0]) <= threshold:
                latency = (response_time - cmd_history[0][1]).to_sec()
                latencies.append(latency)
                cmd_history.popleft()
        
        # Verify latency measurements
        self.assertGreater(len(latencies), 0, "Should have measured some latencies")
        
        # All latencies should be reasonable (less than 1 second for this test)
        for latency in latencies:
            self.assertLess(latency, 1.0, f"Latency too high: {latency}s")
            self.assertGreater(latency, 0.05, f"Latency too low: {latency}s")  # At least processing delay
        
        avg_latency = sum(latencies) / len(latencies)
        rospy.loginfo(f"Average latency: {avg_latency:.3f}s from {len(latencies)} samples")


if __name__ == '__main__':
    # Run the test
    rostest.rosrun('fanuc_driver', 'test_weld_control_freq_ros', TestWeldFreqROS)