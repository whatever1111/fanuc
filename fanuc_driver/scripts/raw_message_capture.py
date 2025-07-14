#!/usr/bin/env python3
"""
Debug script to capture and analyze raw welding state messages from Fanuc robot.
This script connects directly to the robot's TCP port and captures raw message data.
"""

import socket
import struct
import time
import sys
import argparse

def capture_raw_messages(robot_ip, port=11002, timeout=30):
    """Capture raw TCP messages from robot"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)  # 5 second connection timeout
    
    try:
        print(f"Connecting to {robot_ip}:{port}...")
        sock.connect((robot_ip, port))
        print(f"✓ Connected to {robot_ip}:{port}")
        
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < timeout:
            try:
                # Read message header (8 bytes: length + message_type)
                header = sock.recv(8)
                if len(header) < 8:
                    print("Connection closed by robot")
                    break
                    
                # Parse header (little-endian format)
                length, msg_type = struct.unpack('<II', header)
                message_count += 1
                
                print(f"\n--- Message {message_count} ---")
                print(f"Header: Length={length}, Type={msg_type}")
                
                # Read message data
                if length > 0:
                    data = sock.recv(length)
                    if len(data) < length:
                        print(f"Warning: Expected {length} bytes, got {len(data)}")
                    
                    print(f"Data ({len(data)} bytes): {data.hex()}")
                    
                    # If this is a weld state message (type 15)
                    if msg_type == 15:
                        print(">>> WELDING STATE MESSAGE <<<")
                        if len(data) >= 32:
                            # Parse as 8 integers (4 bytes each)
                            values = struct.unpack('<8i', data[:32])
                            print(f"Parsed values:")
                            print(f"  arc_ok = {values[0]} ({'TRUE' if values[0] else 'FALSE'})")
                            print(f"  ready = {values[1]} ({'TRUE' if values[1] else 'FALSE'})")
                            print(f"  stick_err = {values[2]} ({'TRUE' if values[2] else 'FALSE'})")
                            print(f"  general_err = {values[3]} ({'TRUE' if values[3] else 'FALSE'})")
                            print(f"  err_code = {values[4]}")
                            print(f"  act_voltage = {values[5]} (raw)")
                            print(f"  act_current = {values[6]} (raw)")
                            print(f"  act_wire_spd = {values[7]} (raw)")
                            
                            # Apply scaling factors
                            voltage_scaled = values[5] * 100.0 / 32767.0
                            current_scaled = values[6] * 1000.0 / 32767.0
                            wire_speed_scaled = values[7] * 40.0 / 32767.0
                            
                            print(f"Scaled values:")
                            print(f"  Voltage: {voltage_scaled:.1f}V")
                            print(f"  Current: {current_scaled:.0f}A")
                            print(f"  Wire Speed: {wire_speed_scaled:.1f}m/min")
                        else:
                            print(f"Warning: Welding message too short ({len(data)} bytes)")
                    
                    elif msg_type == 10:
                        print(">>> JOINT STATE MESSAGE <<<")
                    elif msg_type == 13:
                        print(">>> ROBOT STATUS MESSAGE <<<")
                    else:
                        print(f">>> UNKNOWN MESSAGE TYPE {msg_type} <<<")
                
                time.sleep(0.01)  # Small delay to avoid overwhelming output
                
            except socket.timeout:
                print("No data received (timeout)")
                continue
            except Exception as e:
                print(f"Error reading message: {e}")
                break
                
    except socket.timeout:
        print(f"✗ Connection timeout to {robot_ip}:{port}")
        return False
    except ConnectionRefusedError:
        print(f"✗ Connection refused to {robot_ip}:{port}")
        print("  - Check if robot is powered on and network is accessible")
        print("  - Verify that ros_state.kl program is running on robot")
        return False
    except Exception as e:
        print(f"✗ Connection error: {e}")
        return False
    finally:
        sock.close()
        print(f"\nCaptured {message_count} messages in {time.time() - start_time:.1f} seconds")
    
    return True

def main():
    parser = argparse.ArgumentParser(description='Capture raw welding state messages from Fanuc robot')
    parser.add_argument('robot_ip', help='IP address of the Fanuc robot controller')
    parser.add_argument('--port', type=int, default=11002, help='TCP port (default: 11002)')
    parser.add_argument('--timeout', type=int, default=30, help='Capture timeout in seconds (default: 30)')
    
    args = parser.parse_args()
    
    print("Fanuc Raw Message Capture Tool")
    print("=" * 40)
    print(f"Target: {args.robot_ip}:{args.port}")
    print(f"Timeout: {args.timeout} seconds")
    print("Press Ctrl+C to stop early\n")
    
    try:
        success = capture_raw_messages(args.robot_ip, args.port, args.timeout)
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\nCapture interrupted by user")
        return 0

if __name__ == '__main__':
    sys.exit(main()) 