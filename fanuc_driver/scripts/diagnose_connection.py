#!/usr/bin/env python3
"""
Connection diagnostic tool for Fanuc robot.
Helps identify connection issues and provides solutions.
"""

import socket
import subprocess
import sys
import time


def print_section(title):
    """Print formatted section header"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)


def check_ping(ip):
    """Check if IP is reachable"""
    print_section(f"1. PING TEST - {ip}")
    
    try:
        result = subprocess.run(
            ["ping", "-c", "2", "-W", "2", ip],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print(f"✅ SUCCESS: {ip} is reachable")
            # Parse ping output for latency
            lines = result.stdout.split('\n')
            for line in lines:
                if 'rtt min/avg/max' in line:
                    print(f"   Latency: {line.split('=')[1].strip()}")
            return True
        else:
            print(f"❌ FAILED: Cannot reach {ip}")
            print("   Possible causes:")
            print("   - Wrong IP address")
            print("   - Robot controller is powered off")
            print("   - Network cable disconnected")
            print("   - Firewall blocking ICMP")
            return False
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False


def check_port(ip, port, timeout=3):
    """Check if a specific port is open"""
    print_section(f"2. PORT TEST - {ip}:{port}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    
    try:
        result = sock.connect_ex((ip, port))
        if result == 0:
            print(f"✅ SUCCESS: Port {port} is open")
            return True
        else:
            print(f"❌ FAILED: Port {port} is closed or filtered")
            return False
    except socket.gaierror:
        print(f"❌ ERROR: Hostname could not be resolved")
        return False
    except socket.error as e:
        print(f"❌ ERROR: {e}")
        return False
    finally:
        sock.close()


def check_simple_message(ip, port):
    """Try to establish SimpleMessage connection"""
    print_section(f"3. SIMPLE MESSAGE TEST - {ip}:{port}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    
    try:
        print(f"   Attempting connection to {ip}:{port}...")
        sock.connect((ip, port))
        print("✅ TCP connection established")
        
        # Try to receive any data (even if it's an error)
        sock.settimeout(2)
        try:
            data = sock.recv(1024)
            if data:
                print(f"✅ Received data: {len(data)} bytes")
                print(f"   First 20 bytes (hex): {data[:20].hex()}")
            else:
                print("⚠️  Connected but no data received")
        except socket.timeout:
            print("⚠️  Connected but no data received (timeout)")
            
        return True
        
    except socket.timeout:
        print("❌ Connection timeout")
        return False
    except ConnectionRefusedError:
        print("❌ Connection refused")
        print("   The port is closed or no service is listening")
        return False
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False
    finally:
        sock.close()


def scan_common_ports(ip):
    """Scan common industrial robot ports"""
    print_section(f"4. PORT SCAN - Common Robot Ports on {ip}")
    
    common_ports = {
        11000: "Motion/Command Port (ROS-Industrial)",
        11001: "System Port (ROS-Industrial)",
        11002: "State Port (ROS-Industrial)",
        11003: "I/O Port (ROS-Industrial)",
        80: "HTTP Web Interface",
        21: "FTP",
        23: "Telnet",
        502: "Modbus TCP",
        2000: "Some Fanuc services",
        8193: "Fanuc Web Server",
        60008: "Fanuc KAREL",
    }
    
    open_ports = []
    for port, description in common_ports.items():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        result = sock.connect_ex((ip, port))
        if result == 0:
            print(f"   ✅ Port {port:5} OPEN - {description}")
            open_ports.append(port)
        sock.close()
    
    if not open_ports:
        print("   ❌ No common ports are open")
    
    return open_ports


def check_network_interface():
    """Check local network configuration"""
    print_section("5. LOCAL NETWORK CONFIGURATION")
    
    try:
        result = subprocess.run(
            ["ip", "addr", "show"],
            capture_output=True,
            text=True
        )
        
        lines = result.stdout.split('\n')
        interfaces = []
        current_if = None
        
        for line in lines:
            if ': ' in line and 'inet ' not in line:
                parts = line.split(': ')
                if len(parts) >= 2:
                    current_if = parts[1].split('@')[0]
            elif 'inet ' in line and current_if:
                ip = line.strip().split()[1].split('/')[0]
                if not ip.startswith('127.'):
                    interfaces.append((current_if, ip))
        
        print("   Local network interfaces:")
        for iface, ip in interfaces:
            print(f"   - {iface}: {ip}")
            
        # Check if we're on the same subnet
        for iface, ip in interfaces:
            if ip.startswith('192.168.0.'):
                print(f"\n   ✅ Interface {iface} is on the same subnet as robot")
                return True
                
        print("\n   ⚠️  No interface found on 192.168.0.x subnet")
        print("   You may need to configure your network adapter")
        
    except Exception as e:
        print(f"   Error checking network: {e}")
    
    return False


def provide_solutions(robot_ip):
    """Provide troubleshooting solutions"""
    print_section("TROUBLESHOOTING RECOMMENDATIONS")
    
    print("""
1. **On the Fanuc Robot Controller:**
   - Verify that the KAREL programs are loaded and running:
     * ROS_RELAY (port 11000 for motion)
     * ROS_STATE (port 11002 for state)
   - Check robot teach pendant for any error messages
   - Ensure robot is in AUTO mode (not TEACH)
   - Check controller network settings:
     * Menu → SETUP → Host Comm → TCP/IP

2. **KAREL Program Status:**
   - On teach pendant: Menu → STATUS → Program
   - Look for:
     * ROS_RELAY - should be RUNNING
     * ROS_STATE - should be RUNNING
   - If ABORTED, try:
     * SELECT → choose program → SHIFT+RESET → SHIFT+FWD

3. **Network Configuration:**
   - Robot IP: Verify it's really {robot_ip}
   - Subnet mask: Should match your PC (usually 255.255.255.0)
   - Try direct cable connection if using a switch/router

4. **Firewall Issues:**
   - Windows: Check Windows Defender Firewall
   - Linux: Check iptables/ufw rules
   - Try temporarily disabling firewall for testing

5. **Alternative Ports:**
   - Some Fanuc setups use different ports
   - Try ports: 60008, 2000, or custom configured ports

6. **Test with Telnet/FTP:**
   - Try: telnet {robot_ip}
   - Or: ftp {robot_ip}
   - If these work, network is OK, issue is with ROS programs
""".format(robot_ip=robot_ip))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 diagnose_connection.py <robot_ip> [port]")
        print("Example: python3 diagnose_connection.py 192.168.0.99 11002")
        sys.exit(1)
    
    robot_ip = sys.argv[1]
    robot_port = int(sys.argv[2]) if len(sys.argv) > 2 else 11002
    
    print("\n" + "="*60)
    print("  FANUC ROBOT CONNECTION DIAGNOSTIC")
    print("="*60)
    print(f"Target: {robot_ip}:{robot_port}")
    print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Run diagnostics
    ping_ok = check_ping(robot_ip)
    
    if ping_ok:
        port_ok = check_port(robot_ip, robot_port)
        
        if port_ok:
            check_simple_message(robot_ip, robot_port)
        else:
            # Scan for other open ports
            open_ports = scan_common_ports(robot_ip)
            
            if open_ports:
                print(f"\n⚠️  Port {robot_port} is not open, but other ports are available")
                print("   The ROS programs may not be running on the controller")
            else:
                print("\n❌ No ports are open on the robot controller")
                print("   The controller may have network services disabled")
    
    check_network_interface()
    provide_solutions(robot_ip)
    
    print("\n" + "="*60)
    print("  DIAGNOSTIC COMPLETE")
    print("="*60)


if __name__ == '__main__':
    main()