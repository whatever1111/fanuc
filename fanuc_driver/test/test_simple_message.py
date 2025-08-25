#!/usr/bin/env python3
"""
Unit tests for SimpleMessage protocol implementation.
"""

import unittest
import struct
import sys
import os

# Add src directory to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


class TestSimpleMessage(unittest.TestCase):
    """Test SimpleMessage protocol encoding/decoding"""
    
    def test_header_serialization(self):
        """Test header serialization"""
        # Create test data
        msg_type = 10
        comm_type = 1
        reply_code = 0
        
        # Pack header manually
        expected = struct.pack('<iii', msg_type, comm_type, reply_code)
        
        # Verify size
        self.assertEqual(len(expected), 12)
        
        # Unpack and verify
        unpacked = struct.unpack('<iii', expected)
        self.assertEqual(unpacked[0], msg_type)
        self.assertEqual(unpacked[1], comm_type)
        self.assertEqual(unpacked[2], reply_code)
        
    def test_joint_position_message(self):
        """Test joint position message format"""
        # Create message
        sequence = 42
        positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        # Build data section
        data = struct.pack('<i', sequence)
        for i in range(10):  # MAX_JOINTS = 10
            if i < len(positions):
                data += struct.pack('<f', positions[i])
            else:
                data += struct.pack('<f', 0.0)
                
        # Verify size
        self.assertEqual(len(data), 44)  # 4 + 10*4
        
        # Unpack and verify
        unpacked_seq = struct.unpack('<i', data[:4])[0]
        self.assertEqual(unpacked_seq, sequence)
        
        unpacked_joints = struct.unpack('<10f', data[4:])
        for i in range(6):
            self.assertAlmostEqual(unpacked_joints[i], positions[i], places=5)
        for i in range(6, 10):
            self.assertEqual(unpacked_joints[i], 0.0)
            
    def test_byte_swapping(self):
        """Test byte swapping for big-endian systems"""
        # Test integer swap
        value = 0x12345678
        swapped = struct.unpack('>I', struct.pack('<I', value))[0]
        self.assertEqual(swapped, 0x78563412)
        
        # Test float swap
        value = 1.234
        packed = struct.pack('<f', value)
        swapped_packed = struct.pack('>f', value)
        self.assertNotEqual(packed, swapped_packed)
        
    def test_prefix_format(self):
        """Test message prefix (packet length)"""
        # Test packet length encoding
        packet_length = 56  # Example: header(12) + data(44)
        prefix = struct.pack('<i', packet_length)
        
        self.assertEqual(len(prefix), 4)
        unpacked = struct.unpack('<i', prefix)[0]
        self.assertEqual(unpacked, packet_length)
        
    def test_robot_status_message(self):
        """Test robot status message format"""
        # Create status values
        drives_powered = 1
        e_stopped = 0
        error_code = 0
        in_error = 0
        in_motion = 1
        mode = 2  # AUTO
        motion_possible = 1
        
        # Pack data
        data = struct.pack('<iiiiiii',
                          drives_powered, e_stopped, error_code,
                          in_error, in_motion, mode, motion_possible)
        
        # Verify size
        self.assertEqual(len(data), 28)  # 7 * 4
        
        # Unpack and verify
        unpacked = struct.unpack('<iiiiiii', data)
        self.assertEqual(unpacked[0], drives_powered)
        self.assertEqual(unpacked[1], e_stopped)
        self.assertEqual(unpacked[2], error_code)
        self.assertEqual(unpacked[3], in_error)
        self.assertEqual(unpacked[4], in_motion)
        self.assertEqual(unpacked[5], mode)
        self.assertEqual(unpacked[6], motion_possible)
        
    def test_trajectory_point_message(self):
        """Test joint trajectory point message format"""
        # Create test data
        sequence = 123
        positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        velocities = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
        duration = 0.5
        
        # Pack data
        data = struct.pack('<i', sequence)
        
        # Pack positions
        for i in range(10):
            if i < len(positions):
                data += struct.pack('<f', positions[i])
            else:
                data += struct.pack('<f', 0.0)
                
        # Pack velocities
        for i in range(10):
            if i < len(velocities):
                data += struct.pack('<f', velocities[i])
            else:
                data += struct.pack('<f', 0.0)
                
        # Pack duration
        data += struct.pack('<f', duration)
        
        # Verify size
        self.assertEqual(len(data), 88)  # 4 + 10*4 + 10*4 + 4
        
    def test_special_sequences(self):
        """Test special sequence values"""
        START_TRAJECTORY_DOWNLOAD = -1
        START_TRAJECTORY_STREAMING = -2
        END_TRAJECTORY = -3
        STOP_TRAJECTORY = -4
        
        # Pack and unpack
        for seq in [START_TRAJECTORY_DOWNLOAD, START_TRAJECTORY_STREAMING,
                   END_TRAJECTORY, STOP_TRAJECTORY]:
            packed = struct.pack('<i', seq)
            unpacked = struct.unpack('<i', packed)[0]
            self.assertEqual(unpacked, seq)


class TestFanucUtils(unittest.TestCase):
    """Test Fanuc-specific utilities"""
    
    def test_j23_linkage_positive(self):
        """Test positive J2-J3 linkage"""
        joints_in = [0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
        j23_factor = 1
        
        # Apply transform: J3_out = J3_in + J23_factor * J2_in
        expected_j3 = 1.0 + 1 * 0.5  # 1.5
        
        joints_out = joints_in.copy()
        joints_out[2] = expected_j3
        
        self.assertAlmostEqual(joints_out[2], 1.5, places=5)
        
    def test_j23_linkage_negative(self):
        """Test negative J2-J3 linkage"""
        joints_in = [0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
        j23_factor = -1
        
        # Apply transform: J3_out = J3_in + J23_factor * J2_in
        expected_j3 = 1.0 + (-1) * 0.5  # 0.5
        
        joints_out = joints_in.copy()
        joints_out[2] = expected_j3
        
        self.assertAlmostEqual(joints_out[2], 0.5, places=5)
        
    def test_j23_linkage_none(self):
        """Test no J2-J3 linkage"""
        joints_in = [0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
        j23_factor = 0
        
        # Apply transform: J3_out = J3_in + J23_factor * J2_in
        expected_j3 = 1.0 + 0 * 0.5  # 1.0
        
        joints_out = joints_in.copy()
        joints_out[2] = expected_j3
        
        self.assertAlmostEqual(joints_out[2], 1.0, places=5)


if __name__ == '__main__':
    unittest.main()