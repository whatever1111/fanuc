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
        
        # Wait for publisher to be ready
        rospy.sleep(1.0)
        rospy.loginfo("Weld Command Tester initialized")
    
    def send_command(self, **kwargs):
        """Send a welding command with specified parameters"""
        cmd = WeldCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "weld_command"
        
        # Set values from arguments, defaulting to the current message value (0) if not provided
        cmd.target_wire_spd = kwargs.get('wire_speed', cmd.target_wire_spd)
        cmd.correction_val = kwargs.get('correction', cmd.correction_val)
        cmd.dyn_setting = kwargs.get('dynamic', cmd.dyn_setting)
        cmd.operation_mode = kwargs.get('mode', cmd.operation_mode)
        cmd.std_pulse_val = kwargs.get('std_pulse', cmd.std_pulse_val)
        cmd.program_number = kwargs.get('program', cmd.program_number)
        
        cmd.arc_start_cmd = kwargs.get('arc_start', cmd.arc_start_cmd)
        cmd.gas_control = kwargs.get('gas', cmd.gas_control)
        cmd.jog_feed_cmd = kwargs.get('jog_feed', cmd.jog_feed_cmd)
        cmd.jog_retract_cmd = kwargs.get('jog_retract', cmd.jog_retract_cmd)
        
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
                if val_str == '' and default is not None:
                    return default
                try:
                    return int(val_str)
                except (ValueError, TypeError):
                    return default

            params['wire_speed'] = get_int("Wire Speed (0-100): ", 0)
            params['correction'] = get_int("Correction (-999 to +999): ", 0)
            params['dynamic'] = get_int("Dynamic (0-100): ", 0)
            params['mode'] = get_int("Operation Mode (0=Std, 1=Pulse): ", 0)
            params['std_pulse'] = get_int("Std/Pulse Value (0-100): ", 0)
            params['program'] = get_int("Program Number (1-99): ", 0)
            
            print("\nDigital Outputs (0=OFF, 1=ON):")
            params['arc_start'] = get_int("Arc Start (DO[253]): ", 0)
            params['gas'] = get_int("Gas Control (DO[255]): ", 0)
            params['jog_feed'] = get_int("Jog Feed (DO[257]): ", 0)
            params['jog_retract'] = get_int("Jog Retract (DO[259]): ", 0)
            
            self.send_command(**params)
            
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
    parser.add_argument('--wire_speed', type=int, help='Wire feed speed (GO[3])')
    parser.add_argument('--correction', type=int, help='Correction value (GO[4])')
    parser.add_argument('--dynamic', type=int, help='Dynamic setting (GO[5])')
    parser.add_argument('--mode', type=int, help='Operation mode (GO[6]): 0=Standard, 1=Pulse')
    parser.add_argument('--std_pulse', type=int, help='Standard/Pulse value (GO[7])')
    parser.add_argument('--program', type=int, help='Program number (GO[2])')
    
    # Digital Output parameters (On/Off pairs)
    parser.add_argument('--arc_start', action='store_true', help='Turn ON arc start (DO[253])')
    parser.add_argument('--arc_stop', action='store_true', help='Turn OFF arc start (DO[253])')
    
    parser.add_argument('--gas_on', action='store_true', help='Turn ON gas (DO[255])')
    parser.add_argument('--gas_off', action='store_true', help='Turn OFF gas (DO[255])')

    parser.add_argument('--jog_feed_on', action='store_true', help='Turn ON jog wire feed (DO[257])')
    parser.add_argument('--jog_feed_off', action='store_true', help='Turn OFF jog wire feed (DO[257])')

    parser.add_argument('--jog_retract_on', action='store_true', help='Turn ON jog wire retract (DO[259])')
    parser.add_argument('--jog_retract_off', action='store_true', help='Turn OFF jog wire retract (DO[259])')
    
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

            if args.arc_start: params['arc_start'] = 1
            if args.arc_stop: params['arc_start'] = 0

            if args.gas_on: params['gas'] = 1
            if args.gas_off: params['gas'] = 0

            if args.jog_feed_on: params['jog_feed'] = 1
            if args.jog_feed_off: params['jog_feed'] = 0

            if args.jog_retract_on: params['jog_retract'] = 1
            if args.jog_retract_off: params['jog_retract'] = 0
            
            if not params and not any([
                args.arc_start, args.arc_stop, args.gas_on, args.gas_off,
                args.jog_feed_on, args.jog_feed_off, args.jog_retract_on, args.jog_retract_off
            ]):
                parser.print_help()
                rospy.logwarn("No command specified. Use --interactive or provide command arguments.")
                return

            tester.send_command(**params)
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == '__main__':
    main() 