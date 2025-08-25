#!/usr/bin/env python3
"""
Test for Fanuc robot state node.
Tests SimpleMessage protocol reception and joint state publishing.
"""

import unittest
import socket
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from fanuc_driver.msg import RobotStatus, JointPosition
import launch
import launch_ros
import launch_testing
import launch_testing.actions
import pytest


class SimpleMessageServer:
    """Mock SimpleMessage server for testing"""
    
    def __init__(self, port=11002, use_bswap=False):
        self.port = port
        self.use_bswap = use_bswap
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.thread = None
        
    def start(self):
        """Start the mock server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('127.0.0.1', self.port))
        self.server_socket.listen(1)
        self.running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.start()
        
    def _run(self):
        """Server thread"""
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                self.client_socket, addr = self.server_socket.accept()
                break
            except socket.timeout:
                continue
                
    def send_joint_position(self, positions, sequence=1):
        """Send a joint position message"""
        if not self.client_socket:
            return False
            
        # Build SimpleMessage packet
        # Prefix (4 bytes): packet length
        # Header (12 bytes): msg_type, comm_type, reply_code
        # Data: sequence (4 bytes) + 10 joints (40 bytes)
        
        msg_type = 10  # JOINT_POSITION
        comm_type = 1  # TOPIC
        reply_code = 0  # INVALID
        
        # Pack data
        data = struct.pack('<i', sequence)  # sequence
        for i in range(10):  # 10 joints max
            if i < len(positions):
                data += struct.pack('<f', positions[i])
            else:
                data += struct.pack('<f', 0.0)
                
        # Pack header
        header = struct.pack('<iii', msg_type, comm_type, reply_code)
        
        # Pack prefix
        packet_length = len(header) + len(data)
        prefix = struct.pack('<i', packet_length)
        
        # Send complete packet
        packet = prefix + header + data
        self.client_socket.sendall(packet)
        return True
        
    def send_robot_status(self, mode=2, e_stopped=0, drives_powered=1, 
                          motion_possible=1, in_motion=0, in_error=0, error_code=0):
        """Send a robot status message"""
        if not self.client_socket:
            return False
            
        msg_type = 13  # STATUS
        comm_type = 1  # TOPIC
        reply_code = 0  # INVALID
        
        # Pack data (7 int32 fields)
        data = struct.pack('<iiiiiii', 
                          drives_powered, e_stopped, error_code,
                          in_error, in_motion, mode, motion_possible)
        
        # Pack header
        header = struct.pack('<iii', msg_type, comm_type, reply_code)
        
        # Pack prefix
        packet_length = len(header) + len(data)
        prefix = struct.pack('<i', packet_length)
        
        # Send complete packet
        packet = prefix + header + data
        self.client_socket.sendall(packet)
        return True
        
    def stop(self):
        """Stop the mock server"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        if self.thread:
            self.thread.join()


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for testing"""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='fanuc_driver',
            executable='robot_state_node',
            name='fanuc_robot_state',
            parameters=[{
                'robot_ip': '127.0.0.1',
                'robot_port': 11002,
                'J23_factor': 0,
                'use_bswap': False,
                'joint_names': ['joint_1', 'joint_2', 'joint_3', 
                               'joint_4', 'joint_5', 'joint_6']
            }]
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestRobotStateNode(unittest.TestCase):
    """Test cases for robot state node"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class"""
        rclpy.init()
        cls.mock_server = SimpleMessageServer(port=11002)
        cls.mock_server.start()
        time.sleep(2)  # Wait for server to start
        
    @classmethod
    def tearDownClass(cls):
        """Tear down test class"""
        cls.mock_server.stop()
        rclpy.shutdown()
        
    def setUp(self):
        """Set up each test"""
        self.node = Node('test_node')
        self.joint_states_received = []
        self.robot_status_received = []
        
        # Subscribe to topics
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            lambda msg: self.joint_states_received.append(msg),
            10
        )
        
        self.robot_status_sub = self.node.create_subscription(
            RobotStatus,
            '/robot_status',
            lambda msg: self.robot_status_received.append(msg),
            10
        )
        
    def tearDown(self):
        """Tear down each test"""
        self.node.destroy_node()
        
    def test_joint_position_reception(self):
        """Test receiving joint positions"""
        # Send test joint positions
        test_positions = [0.0, 0.5, 1.0, -0.5, 0.0, 1.57]
        self.mock_server.send_joint_position(test_positions, sequence=100)
        
        # Spin and wait for message
        timeout = time.time() + 5.0
        while time.time() < timeout and len(self.joint_states_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Verify message received
        self.assertGreater(len(self.joint_states_received), 0)
        msg = self.joint_states_received[0]
        
        # Check joint values
        self.assertEqual(len(msg.position), 6)
        for i in range(6):
            self.assertAlmostEqual(msg.position[i], test_positions[i], places=4)
            
    def test_robot_status_reception(self):
        """Test receiving robot status"""
        # Send test status
        self.mock_server.send_robot_status(
            mode=2,  # AUTO
            e_stopped=0,  # Not e-stopped
            drives_powered=1,  # Powered
            motion_possible=1,  # Ready
            in_motion=0,  # Not moving
            in_error=0,  # No error
            error_code=0
        )
        
        # Spin and wait for message
        timeout = time.time() + 5.0
        while time.time() < timeout and len(self.robot_status_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Verify message received
        self.assertGreater(len(self.robot_status_received), 0)
        msg = self.robot_status_received[0]
        
        # Check status values
        self.assertEqual(msg.mode, 2)
        self.assertEqual(msg.e_stopped, 0)
        self.assertEqual(msg.drives_powered, 1)
        self.assertEqual(msg.motion_possible, 1)
        self.assertEqual(msg.in_motion, 0)
        self.assertEqual(msg.in_error, 0)
        self.assertEqual(msg.error_code, 0)
        
    def test_j23_linkage_transform(self):
        """Test J2-J3 linkage transformation"""
        # This would require restarting the node with J23_factor != 0
        # and verifying the transformation is applied
        pass  # TODO: Implement with parameterized test


if __name__ == '__main__':
    unittest.main()