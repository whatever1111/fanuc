#!/usr/bin/env python3
"""
Fanuc Welding Command Test Script
=================================

This script provides a convenient way to test the welding command system.
It can send various welding commands to the robot and monitor the results.

Usage:
    # Send a basic welding command
    ./weld_command_test.py --wire-speed 50 --program 1 --mode 0
    
    # Start arc welding
    ./weld_command_test.py --arc-start --gas-on --wire-speed 45
    
    # Stop welding
    ./weld_command_test.py --arc-stop --gas-off
    
    # Interactive mode
    ./weld_command_test.py --interactive

Author: YOUR_NAME
Date: 2024
"""

import rospy
import argparse
import sys
from fanuc_driver.msg import WeldCommand
from std_msgs.msg import Header

class WeldCommandTester:
    def __init__(self):
        rospy.init_node('weld_command_tester', anonymous=True)
        self.cmd_pub = rospy.Publisher('/weld_command', WeldCommand, queue_size=1)
        
        # Wait for publisher to be ready
        rospy.sleep(1.0)
        rospy.loginfo("Weld Command Tester initialized")
    
    def send_command(self, **kwargs):
        """Send a welding command with specified parameters"""
        cmd = WeldCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "weld_command"
        
        # Set default values
        cmd.target_wire_spd = kwargs.get('wire_speed', 0)      # GO[3]
        cmd.correction_val = kwargs.get('correction', 0)       # GO[4]
        cmd.dyn_setting = kwargs.get('dynamic', 0)             # GO[5]
        cmd.operation_mode = kwargs.get('mode', 0)             # GO[6]
        cmd.std_pulse_val = kwargs.get('std_pulse', 0)         # GO[7]
        cmd.program_number = kwargs.get('program', 0)          # GO[2]
        
        cmd.arc_start_cmd = kwargs.get('arc_start', 0)         # DO[253]
        cmd.gas_control = kwargs.get('gas', 0)                 # DO[255]
        cmd.jog_feed_cmd = kwargs.get('jog_feed', 0)           # DO[257]
        cmd.jog_retract_cmd = kwargs.get('jog_retract', 0)     # DO[259]
        
        # Log the command
        rospy.loginfo("Sending welding command:")
        rospy.loginfo("  Wire Speed: %d (GO[3])", cmd.target_wire_spd)
        rospy.loginfo("  Correction: %d (GO[4])", cmd.correction_val)
        rospy.loginfo("  Dynamic: %d (GO[5])", cmd.dyn_setting)
        rospy.loginfo("  Mode: %d (GO[6])", cmd.operation_mode)
        rospy.loginfo("  Std/Pulse: %d (GO[7])", cmd.std_pulse_val)
        rospy.loginfo("  Program: %d (GO[2])", cmd.program_number)
        rospy.loginfo("  Arc Start: %d (DO[253])", cmd.arc_start_cmd)
        rospy.loginfo("  Gas: %d (DO[255])", cmd.gas_control)
        rospy.loginfo("  Jog Feed: %d (DO[257])", cmd.jog_feed_cmd)
        rospy.loginfo("  Jog Retract: %d (DO[259])", cmd.jog_retract_cmd)
        
        self.cmd_pub.publish(cmd)
        rospy.loginfo("Command sent!")
        rospy.sleep(0.5)  # Give time for command to be processed
    
    def preset_commands(self):
        """Provide some preset welding commands"""
        presets = {
            '1': {
                'name': 'Basic Welding Setup',
                'params': {
                    'wire_speed': 50, 'program': 1, 'mode': 0,
                    'correction': 0, 'dynamic': 50
                }
            },
            '2': {
                'name': 'Start Arc Welding',
                'params': {
                    'wire_speed': 45, 'arc_start': 1, 'gas': 1,
                    'program': 2, 'mode': 0
                }
            },
            '3': {
                'name': 'Stop Welding',
                'params': {
                    'wire_speed': 0, 'arc_start': 0, 'gas': 0
                }
            },
            '4': {
                'name': 'Jog Wire Feed Forward',
                'params': {
                    'jog_feed': 1, 'jog_retract': 0
                }
            },
            '5': {
                'name': 'Jog Wire Feed Backward',
                'params': {
                    'jog_feed': 0, 'jog_retract': 1
                }
            },
            '6': {
                'name': 'Stop Jog',
                'params': {
                    'jog_feed': 0, 'jog_retract': 0
                }
            },
            '7': {
                'name': 'Pulse Mode Setup',
                'params': {
                    'wire_speed': 40, 'mode': 1, 'std_pulse': 75,
                    'program': 3, 'correction': 10
                }
            }
        }
        return presets
    
    def interactive_mode(self):
        """Run in interactive mode for testing"""
        rospy.loginfo("=== Fanuc Welding Command Interactive Test ===")
        
        presets = self.preset_commands()
        
        while not rospy.is_shutdown():
            print("\nAvailable commands:")
            for key, preset in presets.items():
                print(f"  {key}: {preset['name']}")
            print("  c: Custom command")
            print("  q: Quit")
            
            choice = input("\nSelect command (1-7, c, q): ").strip().lower()
            
            if choice == 'q':
                break
            elif choice in presets:
                self.send_command(**presets[choice]['params'])
            elif choice == 'c':
                self.custom_command()
            else:
                print("Invalid choice!")
    
    def custom_command(self):
        """Allow user to input custom command parameters"""
        print("\n=== Custom Command ===")
        try:
            params = {}
            params['wire_speed'] = int(input("Wire Speed (0-100): ") or "0")
            params['correction'] = int(input("Correction (-999 to +999): ") or "0")
            params['dynamic'] = int(input("Dynamic (0-100): ") or "0")
            params['mode'] = int(input("Operation Mode (0=Std, 1=Pulse): ") or "0")
            params['std_pulse'] = int(input("Std/Pulse Value (0-100): ") or "0")
            params['program'] = int(input("Program Number (1-99): ") or "0")
            
            print("\nDigital Outputs (0=OFF, 1=ON):")
            params['arc_start'] = int(input("Arc Start (DO[253]): ") or "0")
            params['gas'] = int(input("Gas Control (DO[255]): ") or "0")
            params['jog_feed'] = int(input("Jog Feed (DO[257]): ") or "0")
            params['jog_retract'] = int(input("Jog Retract (DO[259]): ") or "0")
            
            self.send_command(**params)
            
        except ValueError:
            print("Invalid input! Please enter numbers only.")
        except KeyboardInterrupt:
            print("\nCancelled.")

def main():
    parser = argparse.ArgumentParser(description='Fanuc Welding Command Test Tool')
    
    # Group Output parameters
    parser.add_argument('--wire-speed', type=int, default=0, 
                       help='Wire feed speed (GO[3])')
    parser.add_argument('--correction', type=int, default=0,
                       help='Correction value (GO[4])')
    parser.add_argument('--dynamic', type=int, default=0,
                       help='Dynamic setting (GO[5])')
    parser.add_argument('--mode', type=int, default=0,
                       help='Operation mode (GO[6]): 0=Standard, 1=Pulse')
    parser.add_argument('--std-pulse', type=int, default=0,
                       help='Standard/Pulse value (GO[7])')
    parser.add_argument('--program', type=int, default=0,
                       help='Program number (GO[2])')
    
    # Digital Output parameters
    parser.add_argument('--arc-start', action='store_true',
                       help='Turn on arc start (DO[253])')
    parser.add_argument('--arc-stop', action='store_true',
                       help='Turn off arc start (DO[253])')
    parser.add_argument('--gas-on', action='store_true',
                       help='Turn on gas (DO[255])')
    parser.add_argument('--gas-off', action='store_true',
                       help='Turn off gas (DO[255])')
    parser.add_argument('--jog-feed', action='store_true',
                       help='Start jog wire feed (DO[257])')
    parser.add_argument('--jog-retract', action='store_true',
                       help='Start jog wire retract (DO[259])')
    parser.add_argument('--jog-stop', action='store_true',
                       help='Stop all jog operations')
    
    # Modes
    parser.add_argument('--interactive', action='store_true',
                       help='Run in interactive mode')
    
    args = parser.parse_args()
    
    try:
        tester = WeldCommandTester()
        
        if args.interactive:
            tester.interactive_mode()
        else:
            # Build command parameters
            params = {
                'wire_speed': args.wire_speed,
                'correction': args.correction,
                'dynamic': args.dynamic,
                'mode': args.mode,
                'std_pulse': args.std_pulse,
                'program': args.program,
                'arc_start': 1 if args.arc_start else (0 if args.arc_stop else 0),
                'gas': 1 if args.gas_on else (0 if args.gas_off else 0),
                'jog_feed': 1 if args.jog_feed else (0 if args.jog_stop else 0),
                'jog_retract': 1 if args.jog_retract else (0 if args.jog_stop else 0)
            }
            
            tester.send_command(**params)
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == '__main__':
    main() 