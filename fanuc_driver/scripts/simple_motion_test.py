#!/usr/bin/env python3
"""
Simple command-line tool for testing robot motion.
Usage: ros2 run fanuc_driver simple_motion_test.py [home|test|custom]
"""

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time


class SimpleMotionNode(Node):
    def __init__(self):
        super().__init__('simple_motion_test')
        
        # Robot configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 
                           'joint_4', 'joint_5', 'joint_6']
        
        # Predefined positions (in radians)
        self.positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.785, 1.571, 0.0, 0.785, 0.0],  # Ready position
            'test1': [0.524, -0.524, 0.785, 0.0, 0.524, 0.0],  # Test position 1
            'test2': [-0.524, -0.785, 1.047, 0.0, 0.785, 0.0], # Test position 2
        }
        
        self.current_joint_state = None
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_path_command',
            10
        )
        
        self.get_logger().info('Simple Motion Test initialized')
        
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        
    def move_to_position(self, target_name, duration=3.0):
        """Move robot to a named position"""
        
        if target_name not in self.positions:
            self.get_logger().error(f'Unknown position: {target_name}')
            self.get_logger().info(f'Available positions: {list(self.positions.keys())}')
            return False
            
        # Wait for joint states
        timeout = 5.0
        start = time.time()
        while self.current_joint_state is None:
            if time.time() - start > timeout:
                self.get_logger().error('Timeout waiting for joint states')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Get current and target positions
        current_pos = list(self.current_joint_state.position)
        target_pos = self.positions[target_name]
        
        self.get_logger().info(f'Moving to {target_name} position')
        self.get_logger().info(f'Current: {[f"{p:.3f}" for p in current_pos]}')
        self.get_logger().info(f'Target:  {[f"{p:.3f}" for p in target_pos]}')
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Start point (current position)
        point1 = JointTrajectoryPoint()
        point1.positions = current_pos
        point1.velocities = [0.0] * 6
        point1.time_from_start = Duration(sec=0)
        trajectory.points.append(point1)
        
        # End point (target position)
        point2 = JointTrajectoryPoint()
        point2.positions = target_pos
        point2.velocities = [0.0] * 6
        point2.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1) * 1e9))
        trajectory.points.append(point2)
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Trajectory published, moving for {duration} seconds...')
        
        # Wait for motion to complete
        time.sleep(duration + 0.5)
        self.get_logger().info('Motion completed')
        return True
        
    def move_joints(self, joint_deltas, duration=2.0):
        """Move joints by specified deltas from current position"""
        
        # Wait for joint states
        timeout = 5.0
        start = time.time()
        while self.current_joint_state is None:
            if time.time() - start > timeout:
                self.get_logger().error('Timeout waiting for joint states')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Calculate target position
        current_pos = list(self.current_joint_state.position)
        target_pos = [current_pos[i] + joint_deltas[i] for i in range(6)]
        
        self.get_logger().info('Moving joints by deltas')
        self.get_logger().info(f'Deltas: {[f"{d:.3f}" for d in joint_deltas]}')
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Current position
        point1 = JointTrajectoryPoint()
        point1.positions = current_pos
        point1.velocities = [0.0] * 6
        point1.time_from_start = Duration(sec=0)
        trajectory.points.append(point1)
        
        # Target position
        point2 = JointTrajectoryPoint()
        point2.positions = target_pos
        point2.velocities = [0.0] * 6
        point2.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1) * 1e9))
        trajectory.points.append(point2)
        
        # Publish
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Moving for {duration} seconds...')
        
        time.sleep(duration + 0.5)
        self.get_logger().info('Motion completed')
        return True
        
    def demo_sequence(self):
        """Run a demo sequence of movements"""
        self.get_logger().info('Starting demo sequence...')
        
        moves = [
            ('home', 3.0),
            ('ready', 3.0),
            ('test1', 3.0),
            ('test2', 3.0),
            ('ready', 3.0),
            ('home', 3.0),
        ]
        
        for position, duration in moves:
            if not self.move_to_position(position, duration):
                self.get_logger().error('Demo sequence aborted')
                return False
            time.sleep(1.0)  # Pause between moves
            
        self.get_logger().info('Demo sequence completed')
        return True


def print_usage():
    print("""
Usage: ros2 run fanuc_driver simple_motion_test.py [command]

Commands:
  home              - Move to home position (all zeros)
  ready             - Move to ready position
  test1             - Move to test position 1
  test2             - Move to test position 2
  demo              - Run demo sequence
  j1 <angle>        - Move joint 1 by angle (degrees)
  j2 <angle>        - Move joint 2 by angle (degrees)
  j3 <angle>        - Move joint 3 by angle (degrees)
  j4 <angle>        - Move joint 4 by angle (degrees)
  j5 <angle>        - Move joint 5 by angle (degrees)
  j6 <angle>        - Move joint 6 by angle (degrees)
  
Examples:
  ros2 run fanuc_driver simple_motion_test.py home
  ros2 run fanuc_driver simple_motion_test.py j1 30
  ros2 run fanuc_driver simple_motion_test.py demo
""")


def main():
    if len(sys.argv) < 2:
        print_usage()
        return
        
    command = sys.argv[1].lower()
    
    rclpy.init()
    node = SimpleMotionNode()
    
    try:
        if command == 'home':
            node.move_to_position('home')
        elif command == 'ready':
            node.move_to_position('ready')
        elif command == 'test1':
            node.move_to_position('test1')
        elif command == 'test2':
            node.move_to_position('test2')
        elif command == 'demo':
            node.demo_sequence()
        elif command.startswith('j'):
            # Joint motion command
            if len(sys.argv) < 3:
                print(f"Error: {command} requires an angle argument")
                print_usage()
                return
                
            joint_num = int(command[1]) - 1
            if joint_num < 0 or joint_num > 5:
                print(f"Error: Invalid joint number {command[1]}")
                return
                
            angle_deg = float(sys.argv[2])
            angle_rad = angle_deg * 3.14159 / 180.0
            
            deltas = [0.0] * 6
            deltas[joint_num] = angle_rad
            
            node.get_logger().info(f'Moving joint {joint_num+1} by {angle_deg} degrees')
            node.move_joints(deltas)
        else:
            print(f"Unknown command: {command}")
            print_usage()
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()