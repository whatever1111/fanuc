#!/usr/bin/env python3
"""
Fanuc Welding Command Test Script
=================================

This script provides a convenient way to test the welding command system.
It can send various welding commands to the robot and monitor the results.

Usage:
    # Send a basic welding command
    ./weld_command_test.py --program 1 --wire-speed 50 --mode 0
    
    # Start arc welding
    ./weld_command_test.py --arc-start --gas-on --wire-speed 45 --program 2
    
    # Stop welding
    ./weld_command_test.py --arc-stop --gas-off
    
    # Start wire feeding
    ./weld_command_test.py --jog-feed-on

    # Stop wire feeding
    ./weld_command_test.py --jog-feed-off

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
        
        rospy.loginfo("Waiting for subscriber to connect to /weld_command...")
        wait_start_time = rospy.Time.now()
        
        while self.cmd_pub.get_num_connections() == 0:
            if rospy.is_shutdown():
                sys.exit(-1)
            if rospy.Time.now() - wait_start_time > rospy.Duration(10.0):
                rospy.logerr("Timeout waiting for subscriber. Is the fanuc_weld_command_node running?")
                sys.exit(-1)
            rospy.sleep(0.1)

        rospy.loginfo("Subscriber connected! Weld Command Tester initialized.")
    
    def send_command(self, **kwargs):
        """Send a welding command with specified parameters"""
        cmd = WeldCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "weld_command"
        
        # Default all values to -1 (NO_CHANGE sentinel)
        cmd.target_wire_spd = -1
        cmd.correction_val = -1
        cmd.dyn_setting = -1
        cmd.operation_mode = -1
        cmd.std_pulse_val = -1
        cmd.program_number = -1
        cmd.arc_start_cmd = -1
        cmd.gas_control = -1
        cmd.jog_feed_cmd = -1
        cmd.jog_retract_cmd = -1

        # Set values from arguments, overwriting the -1 default if provided
        if 'wire_speed' in kwargs: cmd.target_wire_spd = kwargs['wire_speed']
        if 'correction' in kwargs: cmd.correction_val = kwargs['correction']
        if 'dynamic' in kwargs: cmd.dyn_setting = kwargs['dynamic']
        if 'mode' in kwargs: cmd.operation_mode = kwargs['mode']
        if 'std_pulse' in kwargs: cmd.std_pulse_val = kwargs['std_pulse']
        if 'program' in kwargs: cmd.program_number = kwargs['program']
        
        if 'arc_start' in kwargs: cmd.arc_start_cmd = kwargs['arc_start']
        if 'gas' in kwargs: cmd.gas_control = kwargs['gas']
        if 'jog_feed' in kwargs: cmd.jog_feed_cmd = kwargs['jog_feed']
        if 'jog_retract' in kwargs: cmd.jog_retract_cmd = kwargs['jog_retract']
        
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
                'name': 'Start Jog Wire Feed',
                'params': {
                    'jog_feed': 1
                }
            },
            '5': {
                'name': 'Stop Jog Wire Feed',
                'params': {
                    'jog_feed': 0
                }
            },
            '6': {
                'name': 'Start Jog Wire Retract',
                'params': {
                    'jog_retract': 1
                }
            },
            '7': {
                'name': 'Stop Jog Wire Retract',
                'params': {
                    'jog_retract': 0
                }
            },
            '8': {
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
            for key, preset in sorted(presets.items()):
                print(f"  {key}: {preset['name']}")
            print("  c: Custom command")
            print("  q: Quit")
            
            choice = input("\nSelect command: ").strip().lower()
            
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
        print("\n=== Custom Command (press Enter to skip a parameter) ===")
        try:
            params = {}
            def get_int(prompt, default=None):
                val_str = input(prompt)
                if val_str == '':
                    return None  # Return None if user skips
                try:
                    return int(val_str)
                except (ValueError, TypeError):
                    return None

            params['wire_speed'] = get_int("Wire Speed (0-100): ")
            params['correction'] = get_int("Correction (-999 to +999): ")
            params['dynamic'] = get_int("Dynamic (0-100): ")
            params['mode'] = get_int("Operation Mode (0=Std, 1=Pulse): ")
            params['std_pulse'] = get_int("Std/Pulse Value (0-100): ")
            params['program'] = get_int("Program Number (1-99): ")
            
            print("\nDigital Outputs (0=OFF, 1=ON):")
            params['arc_start'] = get_int("Arc Start (DO[253]): ")
            params['gas'] = get_int("Gas Control (DO[255]): ")
            params['jog_feed'] = get_int("Jog Feed (DO[257]): ")
            params['jog_retract'] = get_int("Jog Retract (DO[259]): ")
            
            # Filter out None values so they don't overwrite defaults in send_command
            final_params = {k: v for k, v in params.items() if v is not None}
            
            self.send_command(**final_params)
            
        except ValueError:
            print("Invalid input! Please enter numbers only.")
        except KeyboardInterrupt:
            print("\nCancelled.")

def main():
    parser = argparse.ArgumentParser(
        description='Fanuc Welding Command Test Tool. Send specific commands to the welder.',
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # Group Output parameters
    parser.add_argument('--wire_speed', type=int, help='Wire feed speed (GO[3])', dest='wire_speed')
    parser.add_argument('--correction', type=int, help='Correction value (GO[4])', dest='correction')
    parser.add_argument('--dynamic', type=int, help='Dynamic setting (GO[5])', dest='dynamic')
    parser.add_argument('--mode', type=int, help='Operation mode (GO[6]): 0=Standard, 1=Pulse', dest='mode')
    parser.add_argument('--std_pulse', type=int, help='Standard/Pulse value (GO[7])', dest='std_pulse')
    parser.add_argument('--program', type=int, help='Program number (GO[2])', dest='program')
    
    # Digital Output parameters (On/Off pairs)
    parser.add_argument('--arc_start', action='store_const', const=1, dest='arc_start', help='Turn ON arc start (DO[253])')
    parser.add_argument('--arc_stop', action='store_const', const=0, dest='arc_start', help='Turn OFF arc start (DO[253])')
    
    parser.add_argument('--gas_on', action='store_const', const=1, dest='gas', help='Turn ON gas (DO[255])')
    parser.add_argument('--gas_off', action='store_const', const=0, dest='gas', help='Turn OFF gas (DO[255])')

    parser.add_argument('--jog_feed_on', action='store_const', const=1, dest='jog_feed', help='Turn ON jog wire feed (DO[257])')
    parser.add_argument('--jog_feed_off', action='store_const', const=0, dest='jog_feed', help='Turn OFF jog wire feed (DO[257])')

    parser.add_argument('--jog_retract_on', action='store_const', const=1, dest='jog_retract', help='Turn ON jog wire retract (DO[259])')
    parser.add_argument('--jog_retract_off', action='store_const', const=0, dest='jog_retract', help='Turn OFF jog wire retract (DO[259])')
    
    # Modes
    parser.add_argument('--interactive', action='store_true',
                       help='Run in interactive mode to select from preset commands.')
    
    args = parser.parse_args()
    
    try:
        tester = WeldCommandTester()
        
        if args.interactive:
            tester.interactive_mode()
        else:
            # Build command parameters from provided args
            params = {}
            if args.wire_speed is not None: params['wire_speed'] = args.wire_speed
            if args.correction is not None: params['correction'] = args.correction
            if args.dynamic is not None: params['dynamic'] = args.dynamic
            if args.mode is not None: params['mode'] = args.mode
            if args.std_pulse is not None: params['std_pulse'] = args.std_pulse
            if args.program is not None: params['program'] = args.program

            # Consolidate arc_start, gas, etc. from args
            if args.arc_start is not None: params['arc_start'] = args.arc_start
            if args.gas is not None: params['gas'] = args.gas
            if args.jog_feed is not None: params['jog_feed'] = args.jog_feed
            if args.jog_retract is not None: params['jog_retract'] = args.jog_retract
            
            if not params:
                parser.print_help()
                rospy.logwarn("No command specified. Use --interactive or provide command arguments.")
                return

            rospy.loginfo("Command-line args parsed. Parameters to send: %s", params)
            tester.send_command(**params)
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == '__main__':
    main() 