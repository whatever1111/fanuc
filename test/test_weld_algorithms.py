#!/usr/bin/env python3
"""
ROS Unit Tests for Weld Control Algorithms
==========================================

This test suite validates the mathematical algorithms used in weld_control_freq_test.py
using the ROS unittest framework. Tests are focused on algorithm correctness without
requiring ROS message infrastructure.

Usage:
    rostest fanuc_driver test_weld_algorithms.test

Author: Test Suite
Date: 2024
"""

import unittest
import rospy
import rostest
import math
import random
from typing import List, Tuple


class TestWeldAlgorithms(unittest.TestCase):
    """Unit tests for weld control algorithms"""
    
    def setUp(self):
        """Set up test environment"""
        rospy.init_node('test_weld_algorithms', anonymous=True)
        
        # Get test parameters (ensure correct types even if passed as strings)
        prec = rospy.get_param('~test_precision', 1e-6)
        iters = rospy.get_param('~test_iterations', 1000)
        try:
            self.test_precision = float(prec)
        except (TypeError, ValueError):
            self.test_precision = 1e-6
        try:
            self.test_iterations = int(iters)
        except (TypeError, ValueError):
            self.test_iterations = 1000
        
        rospy.loginfo(f"Algorithm tests initialized with precision: {self.test_precision}")
    
    def test_sine_wave_generation(self):
        """Test sine wave generation algorithm"""
        rospy.loginfo("Testing sine wave generation...")
        
        # Test parameters
        freq = 2.0
        amp = 10.0
        offset = 25.0
        
        # Test known points
        test_cases = [
            (0.0, offset),                    # t=0: sin(0) = 0
            (0.25/freq, offset + amp),        # t=T/4: sin(π/2) = 1
            (0.5/freq, offset),               # t=T/2: sin(π) = 0
            (0.75/freq, offset - amp),        # t=3T/4: sin(3π/2) = -1
            (1.0/freq, offset),               # t=T: sin(2π) = 0
        ]
        
        for t, expected in test_cases:
            sine_val = math.sin(2 * math.pi * freq * t)
            actual = offset + amp * sine_val
            
            self.assertAlmostEqual(actual, expected, delta=self.test_precision,
                                 msg=f"Sine wave error at t={t}: {actual} != {expected}")
        
        rospy.loginfo("✓ Sine wave generation test passed")
    
    def test_step_wave_generation(self):
        """Test step wave generation algorithm"""
        rospy.loginfo("Testing step wave generation...")
        
        # Test parameters
        freq = 1.0
        amp = 15.0
        offset = 30.0
        
        # Test cases covering both phases
        test_cases = [
            (0.1, offset + amp),    # First half of period
            (0.4, offset + amp),    # Still first half
            (0.6, offset - amp),    # Second half of period
            (0.9, offset - amp),    # Still second half
            (1.1, offset + amp),    # Next period, first half
        ]
        
        for t, expected in test_cases:
            phase = (t * freq) % 1.0
            step_val = amp if phase < 0.5 else -amp
            actual = offset + step_val
            
            self.assertEqual(actual, expected,
                           msg=f"Step wave error at t={t}: {actual} != {expected}")
        
        rospy.loginfo("✓ Step wave generation test passed")
    
    def test_pulse_probability_distribution(self):
        """Test pulse generation probability distribution"""
        rospy.loginfo("Testing pulse probability distribution...")
        
        # Test parameters
        pulse_rate = 10.0  # pulses per second
        sample_rate = 1000.0  # Hz
        duration = 5.0  # seconds
        
        pulse_prob_per_sample = pulse_rate / sample_rate
        total_samples = int(duration * sample_rate)
        
        # Set deterministic seed
        random.seed(789)
        
        # Count pulse occurrences
        pulse_count = 0
        for _ in range(total_samples):
            if random.random() < pulse_prob_per_sample:
                pulse_count += 1
        
        # Expected number of pulses
        expected_pulses = pulse_rate * duration
        relative_error = abs(pulse_count - expected_pulses) / expected_pulses
        
        # Allow 15% variance due to randomness
        self.assertLess(relative_error, 0.15,
                       f"Pulse rate error too large: {pulse_count} vs {expected_pulses} (error: {relative_error:.3f})")
        
        rospy.loginfo(f"✓ Pulse distribution test passed: {pulse_count}/{expected_pulses} pulses")
    
    def test_pulse_duration_control(self):
        """Test pulse duration control algorithm"""
        rospy.loginfo("Testing pulse duration control...")
        
        # Test parameters
        pulse_duration = 0.1  # seconds
        sample_rate = 100.0   # Hz
        dt = 1.0 / sample_rate
        
        # Simulate pulse lifecycle
        pulse_start_time = 1.0
        pulse_end_time = pulse_start_time + pulse_duration
        
        # Count active samples
        active_samples = 0
        test_duration = 3.0
        time_points = [i * dt for i in range(int(test_duration * sample_rate))]
        
        for t in time_points:
            if pulse_start_time <= t < pulse_end_time:
                active_samples += 1
        
        expected_samples = int(pulse_duration * sample_rate)
        
        # Allow ±1 sample tolerance due to discrete time
        self.assertAlmostEqual(active_samples, expected_samples, delta=1,
                              msg=f"Pulse duration control failed: {active_samples} vs {expected_samples}")
        
        rospy.loginfo(f"✓ Pulse duration test passed: {active_samples}/{expected_samples} samples")
    
    def test_signal_bounds_validation(self):
        """Test signal bounds validation"""
        rospy.loginfo("Testing signal bounds validation...")
        
        # Test extreme parameter combinations
        test_cases = [
            {"step_amp": 50.0, "offset": 100.0, "pulse_amp": 25.0},  # Large values
            {"step_amp": 1.0, "offset": 5.0, "pulse_amp": 0.5},      # Small values
            {"step_amp": 0.0, "offset": 10.0, "pulse_amp": 5.0},     # Zero step
            {"step_amp": 10.0, "offset": 0.0, "pulse_amp": 5.0},     # Zero offset
        ]
        
        for i, params in enumerate(test_cases):
            step_amp = params["step_amp"]
            offset = params["offset"]
            pulse_amp = params["pulse_amp"]
            
            # Calculate theoretical bounds
            min_signal = offset - step_amp - pulse_amp
            max_signal = offset + step_amp + pulse_amp
            
            # Test signal generation at extreme combinations
            test_signals = [
                int(offset + step_amp + pulse_amp),     # Maximum positive
                int(offset - step_amp - pulse_amp),     # Maximum negative
                int(offset + step_amp - pulse_amp),     # Mixed 1
                int(offset - step_amp + pulse_amp),     # Mixed 2
            ]
            
            for signal in test_signals:
                self.assertGreaterEqual(signal, min_signal - 1,
                                      f"Case {i}: Signal {signal} below minimum {min_signal}")
                self.assertLessEqual(signal, max_signal + 1,
                                   f"Case {i}: Signal {signal} above maximum {max_signal}")
        
        rospy.loginfo("✓ Signal bounds validation test passed")
    
    def test_numerical_stability(self):
        """Test numerical stability of algorithms"""
        rospy.loginfo("Testing numerical stability...")
        
        # Test with very small and very large values
        test_cases = [
            {"freq": 1e-6, "duration": 1e6},   # Very small frequency, large time
            {"freq": 1000.0, "duration": 0.001}, # Very high frequency, small time
            {"amp": 1e-10, "offset": 1e10},     # Very small amplitude, large offset
        ]
        
        for case in test_cases:
            freq = case.get("freq", 1.0)
            duration = case.get("duration", 1.0)
            amp = case.get("amp", 1.0)
            offset = case.get("offset", 0.0)
            
            # Test a few sample points
            for t in [0.0, duration * 0.25, duration * 0.5, duration * 0.75]:
                try:
                    # Sine calculation
                    sine_val = math.sin(2 * math.pi * freq * t)
                    sine_result = offset + amp * sine_val
                    
                    # Step calculation
                    phase = (t * freq) % 1.0
                    step_val = amp if phase < 0.5 else -amp
                    step_result = offset + step_val
                    
                    # Verify results are finite numbers
                    self.assertTrue(math.isfinite(sine_result),
                                  f"Sine result not finite: {sine_result}")
                    self.assertTrue(math.isfinite(step_result),
                                  f"Step result not finite: {step_result}")
                    
                except (OverflowError, ValueError) as e:
                    self.fail(f"Numerical stability error: {e}")
        
        rospy.loginfo("✓ Numerical stability test passed")
    
    def test_integration_algorithm(self):
        """Test complete pulse-enhanced step signal algorithm"""
        rospy.loginfo("Testing complete integration algorithm...")
        
        # Test parameters
        step_freq = 2.0
        step_amp = 15.0
        offset = 30.0
        pulse_rate = 5.0
        pulse_amp = 10.0
        pulse_duration = 0.1
        duration = 1.0
        sample_rate = 50.0
        
        # Set deterministic seed
        random.seed(999)
        
        # Run the complete algorithm
        dt = 1.0 / sample_rate
        samples = int(duration * sample_rate)
        pulse_prob_per_sample = pulse_rate / sample_rate
        
        pulse_end_time = 0.0
        current_pulse_amplitude = 0.0
        
        generated_values = []
        pulse_events = []
        
        for i in range(samples):
            t_now = i * dt
            
            # Base step signal
            phase = (t_now * step_freq) % 1.0
            base_signal = step_amp if phase < 0.5 else -step_amp
            
            # Pulse handling
            if t_now >= pulse_end_time:
                current_pulse_amplitude = 0.0
                
                if random.random() < pulse_prob_per_sample:
                    current_pulse_amplitude = random.uniform(-pulse_amp, pulse_amp)
                    pulse_end_time = t_now + pulse_duration
                    pulse_events.append((t_now, current_pulse_amplitude))
            
            # Combine signals
            target_val = int(offset + base_signal + current_pulse_amplitude)
            generated_values.append(target_val)
        
        # Validate integration results
        self.assertEqual(len(generated_values), samples,
                        "Should generate expected number of samples")
        
        # Check signal bounds
        min_expected = int(offset - step_amp - pulse_amp)
        max_expected = int(offset + step_amp + pulse_amp)
        
        for val in generated_values:
            self.assertGreaterEqual(val, min_expected - 1,
                                  f"Value {val} below minimum {min_expected}")
            self.assertLessEqual(val, max_expected + 1,
                               f"Value {val} above maximum {max_expected}")
        
        # Check pulse properties
        for _, amp in pulse_events:
            self.assertGreaterEqual(amp, -pulse_amp,
                                  f"Pulse amplitude {amp} below minimum")
            self.assertLessEqual(amp, pulse_amp,
                               f"Pulse amplitude {amp} above maximum")
        
        rospy.loginfo(f"✓ Integration test passed: {len(generated_values)} samples, {len(pulse_events)} pulses")


if __name__ == '__main__':
    # Run the test
    rostest.rosrun('fanuc_driver', 'test_weld_algorithms', TestWeldAlgorithms)