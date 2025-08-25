#!/usr/bin/env python3
"""
Simple test script for Fanuc robot motion control.
Tests both direct trajectory publishing and FollowJointTrajectory action.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import math
import time


class RobotMotionTest(Node):
    def __init__(self):
        super().__init__('robot_motion_test')
        
        # Joint names (6-axis robot)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 
                           'joint_4', 'joint_5', 'joint_6']
        
        # Current joint state
        self.current_joint_state = None
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for direct trajectory command
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_path_command',
            10
        )
        
        # Action client for FollowJointTrajectory
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory'
        )
        
        self.get_logger().info('Robot Motion Test Node initialized')
        self.get_logger().info('Waiting for joint states...')
        
    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.current_joint_state = msg
        
    def wait_for_joint_states(self, timeout=5.0):
        """Wait for joint state to be available"""
        start_time = time.time()
        while self.current_joint_state is None:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for joint states')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True
        
    def create_trajectory_point(self, positions, time_from_start):
        """Create a trajectory point"""
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)  # Zero velocity for simplicity
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(time_from_start), 
                                        nanosec=int((time_from_start % 1) * 1e9))
        return point
        
    def test_direct_trajectory(self):
        """Test sending trajectory via direct topic publishing"""
        self.get_logger().info('Testing direct trajectory publishing...')
        
        if not self.wait_for_joint_states():
            return False
            
        # Get current position
        current_pos = list(self.current_joint_state.position)
        self.get_logger().info(f'Current position: {[f"{p:.3f}" for p in current_pos]}')
        
        # Create a simple trajectory (move joint 1 by 30 degrees)
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Point 1: Current position
        trajectory.points.append(
            self.create_trajectory_point(current_pos, 0.0)
        )
        
        # Point 2: Move joint 1 by +30 degrees (0.524 rad)
        target_pos = current_pos.copy()
        target_pos[0] += 0.524  # 30 degrees in radians
        trajectory.points.append(
            self.create_trajectory_point(target_pos, 2.0)
        )
        
        # Point 3: Move back to original position
        trajectory.points.append(
            self.create_trajectory_point(current_pos, 4.0)
        )
        
        # Publish trajectory
        self.get_logger().info('Publishing trajectory...')
        self.trajectory_pub.publish(trajectory)
        
        # Wait for motion to complete
        time.sleep(5.0)
        self.get_logger().info('Direct trajectory test completed')
        return True
        
    def test_action_trajectory(self):
        """Test sending trajectory via FollowJointTrajectory action"""
        self.get_logger().info('Testing FollowJointTrajectory action...')
        
        if not self.wait_for_joint_states():
            return False
            
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
            
        # Get current position
        current_pos = list(self.current_joint_state.position)
        self.get_logger().info(f'Current position: {[f"{p:.3f}" for p in current_pos]}')
        
        # Create a trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create a square motion pattern for joint 1 and 2
        points = [
            (current_pos, 0.0),
            ([current_pos[0] + 0.3, current_pos[1] + 0.2] + current_pos[2:], 2.0),
            ([current_pos[0] + 0.3, current_pos[1] - 0.2] + current_pos[2:], 4.0),
            ([current_pos[0] - 0.3, current_pos[1] - 0.2] + current_pos[2:], 6.0),
            ([current_pos[0] - 0.3, current_pos[1] + 0.2] + current_pos[2:], 8.0),
            (current_pos, 10.0)
        ]
        
        for pos, time_from_start in points:
            trajectory.points.append(
                self.create_trajectory_point(pos, time_from_start)
            )
        
        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # Send goal
        self.get_logger().info('Sending trajectory goal...')
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution successful!')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed: {result.error_string}')
            return False
            
    def test_sine_wave_motion(self):
        """Test smooth sine wave motion on multiple joints"""
        self.get_logger().info('Testing sine wave motion...')
        
        if not self.wait_for_joint_states():
            return False
            
        # Get current position
        current_pos = list(self.current_joint_state.position)
        
        # Create sine wave trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Generate 20 points over 10 seconds
        num_points = 20
        total_time = 10.0
        amplitude = 0.2  # radians
        
        for i in range(num_points):
            t = i * total_time / (num_points - 1)
            phase = 2 * math.pi * t / total_time
            
            # Apply sine wave to joints 1, 2, and 3 with phase shift
            positions = current_pos.copy()
            positions[0] = current_pos[0] + amplitude * math.sin(phase)
            positions[1] = current_pos[1] + amplitude * math.sin(phase + math.pi/3)
            positions[2] = current_pos[2] + amplitude * math.sin(phase + 2*math.pi/3)
            
            trajectory.points.append(
                self.create_trajectory_point(positions, t)
            )
        
        # Publish trajectory
        self.get_logger().info('Publishing sine wave trajectory...')
        self.trajectory_pub.publish(trajectory)
        
        # Wait for motion to complete
        time.sleep(total_time + 1.0)
        self.get_logger().info('Sine wave motion completed')
        return True


def main():
    rclpy.init()
    
    test_node = RobotMotionTest()
    
    # Wait for system to be ready
    time.sleep(2.0)
    
    try:
        # Menu for test selection
        while True:
            print("\n" + "="*50)
            print("Fanuc Robot Motion Test")
            print("="*50)
            print("1. Test direct trajectory publishing")
            print("2. Test FollowJointTrajectory action")
            print("3. Test sine wave motion")
            print("4. Run all tests")
            print("0. Exit")
            print("-"*50)
            
            choice = input("Select test (0-4): ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                test_node.test_direct_trajectory()
            elif choice == '2':
                test_node.test_action_trajectory()
            elif choice == '3':
                test_node.test_sine_wave_motion()
            elif choice == '4':
                print("\nRunning all tests...")
                test_node.test_direct_trajectory()
                time.sleep(2.0)
                test_node.test_action_trajectory()
                time.sleep(2.0)
                test_node.test_sine_wave_motion()
            else:
                print("Invalid choice, please try again")
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()