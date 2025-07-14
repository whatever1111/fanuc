# Fanuc Welding State Interface Debug Guide

This guide provides comprehensive debugging strategies for both KAREL and ROS sides of the welding state interface.

## 1. KAREL Side Debugging

### 1.1 Initial Setup Verification

#### A. Check KAREL Program Loading
```karel
-- In teach pendant, verify programs are loaded:
-- Menu → SETUP → Host Comm → Programs
-- Ensure these programs exist:
-- - ros_state.kl
-- - libind_weld_t.kl  
-- - libind_weld_h.kl
```

#### B. Verify Register Mapping
```karel
-- Use teach pendant to check register values:
-- Menu → I/O → Register
-- Check registers R[10], R[12], R[13], R[14]
-- Values should change when welding parameters change
```

### 1.2 Debug KAREL Code with Print Statements

#### A. Add Debug Output to ros_state.kl
```karel
-- Add debug routine to ros_state.kl main loop:
ROUTINE debug_weld_state
VAR
    status_word : INTEGER
    voltage, current, wire_speed : INTEGER
BEGIN
    -- Read raw register values
    status_word := R[WELD_R_STATUS_WORD]
    voltage := R[WELD_R_ACT_VOLT]
    current := R[WELD_R_ACT_CURR]
    wire_speed := R[WELD_R_ACT_WIRE]
    
    -- Print to robot controller log
    WRITE('WELD_DEBUG: Status=', status_word, CR)
    WRITE('WELD_DEBUG: V=', voltage, ' I=', current, ' W=', wire_speed, CR)
    
    -- Print individual status bits
    WRITE('WELD_DEBUG: arc_ok=', (BAnd(status_word, 1) <> 0), CR)
    WRITE('WELD_DEBUG: ready=', (BAnd(status_word, 2) <> 0), CR)
    WRITE('WELD_DEBUG: stick_err=', (BAnd(status_word, 4) <> 0), CR)
    WRITE('WELD_DEBUG: general_err=', (BAnd(status_word, 8) <> 0), CR)
    WRITE('WELD_DEBUG: err_code=', (BAnd(status_word, 65280) DIV 256), CR)
END debug_weld_state
```

#### B. Add Debug Calls to Main Loop
```karel
-- In main loop of ros_state.kl, add:
WHILE (NOT cancel_req) DO
    -- ... existing code ...
    
    -- Add debug output every 10 cycles
    IF (loop_count MOD 10 = 0) THEN
        debug_weld_state
    ENDIF
    
    -- ... existing code ...
ENDWHILE
```

### 1.3 Test Message Serialization

#### A. Create Test Routine for Serialization
```karel
-- Add test routine to verify message packing:
ROUTINE test_weld_serialization
VAR
    test_state : weld_state_t
    test_packet : ind_weld_t
    i : INTEGER
BEGIN
    -- Set known test values
    test_state.arc_ok := TRUE
    test_state.ready := TRUE
    test_state.stick_err := FALSE
    test_state.general_err := FALSE
    test_state.err_code := 42
    test_state.act_voltage := 1000
    test_state.act_current := 2000
    test_state.act_wire_spd := 3000
    
    -- Serialize the data
    iwd_srlise(test_state, test_packet)
    
    -- Print serialized bytes for verification
    WRITE('SERIALIZATION_TEST:', CR)
    FOR i := 1 TO ARRAY_LEN(test_packet.weld_data) DO
        WRITE('Byte[', i, '] = ', test_packet.weld_data[i], CR)
    ENDFOR
END test_weld_serialization
```

### 1.4 Monitor Network Communication

#### A. Check TCP Connection Status
```karel
-- Add connection monitoring to ros_state.kl:
ROUTINE check_tcp_status
VAR
    conn_status : INTEGER
BEGIN
    -- Check if TCP connection is active
    conn_status := TCP_STAT(server_sock)
    WRITE('TCP_STATUS: ', conn_status, CR)
    
    -- Status codes:
    -- 0 = Not connected
    -- 1 = Connected
    -- 2 = Listening
    -- 3 = Error
END check_tcp_status
```

#### B. Monitor Message Sending
```karel
-- Add send monitoring to message sending routine:
ROUTINE send_weld_msg_debug(weld_pkt : ind_weld_t)
VAR
    send_result : INTEGER
BEGIN
    -- Send the message
    send_result := MSG_SEND(RI_MT_WELDST, weld_pkt, client_sock)
    
    -- Log send result
    IF send_result = 0 THEN
        WRITE('WELD_MSG_SENT: OK', CR)
    ELSE
        WRITE('WELD_MSG_SEND_ERROR: ', send_result, CR)
    ENDIF
END send_weld_msg_debug
```

### 1.5 Fieldbus I/O Debugging

#### A. Create I/O Test Program
```karel
PROGRAM io_test_weld
VAR
    i : INTEGER
    reg_val : INTEGER
BEGIN
    -- Continuously monitor welding I/O registers
    FOR i := 1 TO 1000 DO
        WRITE('=== WELD I/O TEST CYCLE ', i, ' ===', CR)
        
        -- Read and display all welding registers
        reg_val := R[WELD_R_STATUS_WORD]
        WRITE('Status Word R[', WELD_R_STATUS_WORD, '] = ', reg_val, CR)
        
        reg_val := R[WELD_R_ACT_VOLT]
        WRITE('Voltage R[', WELD_R_ACT_VOLT, '] = ', reg_val, CR)
        
        reg_val := R[WELD_R_ACT_CURR]
        WRITE('Current R[', WELD_R_ACT_CURR, '] = ', reg_val, CR)
        
        reg_val := R[WELD_R_ACT_WIRE]
        WRITE('Wire Speed R[', WELD_R_ACT_WIRE, '] = ', reg_val, CR)
        
        DELAY 1000  -- Wait 1 second
    ENDFOR
END io_test_weld
```

## 2. ROS Side Debugging

### 2.1 Network Connectivity Testing

#### A. Basic Network Tests
```bash
# Test basic connectivity to robot
ping <ROBOT_IP>

# Test if port 11002 is accessible
telnet <ROBOT_IP> 11002

# Check if port is listening
nmap -p 11002 <ROBOT_IP>
```

#### B. Test TCP Connection
```bash
# Use netcat to test raw TCP connection
nc <ROBOT_IP> 11002

# Monitor network traffic
sudo tcpdump -i eth0 host <ROBOT_IP> and port 11002
```

### 2.2 ROS Node Debugging

#### A. Enable Debug Logging
```bash
# Run weld state node with debug logging
rosrun fanuc_driver weld_state _log_level:=debug robot_ip:=<ROBOT_IP>

# Or with launch file
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false _log_level:=debug
```

#### B. Create Debug Version of C++ Node
```cpp
// Add to fanuc_weld_state_node.cpp for debugging
void processWeldStateMessage(const SimpleMessage& msg)
{
    ByteArray data = msg.getData();
    
    // Debug: Print raw message info
    ROS_DEBUG("Received message - Type: %d, Size: %zu", 
              msg.getMessageType(), data.getBufferSize());
    
    // Debug: Print raw bytes
    if (data.getBufferSize() >= 32)
    {
        ROS_DEBUG("Raw bytes (first 32):");
        for (size_t i = 0; i < 32; ++i)
        {
            ROS_DEBUG("  Byte[%zu] = 0x%02X", i, data.getBuffer()[i]);
        }
    }
    
    // ... rest of existing code ...
    
    // Debug: Print parsed values
    ROS_DEBUG("Parsed values - arc_ok: %d, ready: %d, voltage: %d, current: %d", 
              arc_ok_int, ready_int, act_voltage, act_current);
}
```

### 2.3 Message Analysis Tools

#### A. Raw Message Capture Script
```python
#!/usr/bin/env python3
"""
Debug script to capture and analyze raw welding state messages
"""
import socket
import struct
import time

def capture_raw_messages(robot_ip, port=11002):
    """Capture raw TCP messages from robot"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        sock.connect((robot_ip, port))
        print(f"Connected to {robot_ip}:{port}")
        
        while True:
            # Read message header (8 bytes)
            header = sock.recv(8)
            if len(header) < 8:
                break
                
            # Parse header
            length, msg_type = struct.unpack('<II', header)
            print(f"Message - Length: {length}, Type: {msg_type}")
            
            # Read message data
            if length > 0:
                data = sock.recv(length)
                print(f"Data ({len(data)} bytes): {data.hex()}")
                
                # If this is a weld state message (type 15)
                if msg_type == 15 and len(data) >= 32:
                    # Parse as weld state
                    values = struct.unpack('<8i', data[:32])
                    print(f"Parsed: arc_ok={values[0]}, ready={values[1]}, "
                          f"stick_err={values[2]}, general_err={values[3]}, "
                          f"err_code={values[4]}, voltage={values[5]}, "
                          f"current={values[6]}, wire_speed={values[7]}")
            
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 raw_message_capture.py <ROBOT_IP>")
        sys.exit(1)
    
    capture_raw_messages(sys.argv[1])
```

#### B. Message Validation Script
```python
#!/usr/bin/env python3
"""
Script to validate welding state messages
"""
import rospy
from fanuc_driver.msg import WeldState
import time

class WeldStateValidator:
    def __init__(self):
        self.last_msg_time = None
        self.msg_count = 0
        self.error_count = 0
        
    def validate_message(self, msg):
        """Validate incoming welding state message"""
        current_time = time.time()
        self.msg_count += 1
        
        # Check message timing
        if self.last_msg_time:
            dt = current_time - self.last_msg_time
            if dt > 0.2:  # Expect messages every ~100ms
                rospy.logwarn(f"Message gap too large: {dt:.3f}s")
                self.error_count += 1
        
        self.last_msg_time = current_time
        
        # Validate message content
        if msg.err_code > 255:
            rospy.logwarn(f"Invalid error code: {msg.err_code}")
            self.error_count += 1
            
        # Check for reasonable analog values
        if abs(msg.act_voltage) > 32767 or abs(msg.act_current) > 32767:
            rospy.logwarn(f"Analog values out of range: V={msg.act_voltage}, I={msg.act_current}")
            self.error_count += 1
            
        # Print periodic statistics
        if self.msg_count % 100 == 0:
            error_rate = self.error_count / self.msg_count * 100
            rospy.loginfo(f"Messages: {self.msg_count}, Errors: {self.error_count} ({error_rate:.1f}%)")

def main():
    rospy.init_node('weld_state_validator')
    validator = WeldStateValidator()
    
    def callback(msg):
        validator.validate_message(msg)
    
    sub = rospy.Subscriber('/weld_state', WeldState, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
```

### 2.4 Byte Order Debugging

#### A. Test Both Byte Orders
```bash
# Test normal byte order
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false

# Test byte-swapped order
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=true
```

#### B. Byte Order Detection Script
```python
#!/usr/bin/env python3
"""
Script to help determine correct byte order
"""
import rospy
from fanuc_driver.msg import WeldState

def analyze_byte_order(msg):
    """Analyze message to determine if byte order is correct"""
    
    # Check if values are reasonable
    voltage_scaled = msg.act_voltage * 100.0 / 32767.0
    current_scaled = msg.act_current * 1000.0 / 32767.0
    
    print(f"Raw values: V={msg.act_voltage}, I={msg.act_current}")
    print(f"Scaled: V={voltage_scaled:.1f}V, I={current_scaled:.0f}A")
    
    # Typical welding ranges
    if 10 <= voltage_scaled <= 50 and 50 <= current_scaled <= 500:
        print("✓ Values appear reasonable - byte order likely correct")
    else:
        print("✗ Values out of typical range - check byte order")
        
        # Try swapped interpretation
        voltage_swapped = ((msg.act_voltage << 8) | (msg.act_voltage >> 8)) * 100.0 / 32767.0
        current_swapped = ((msg.act_current << 8) | (msg.act_current >> 8)) * 1000.0 / 32767.0
        print(f"Swapped interpretation: V={voltage_swapped:.1f}V, I={current_swapped:.0f}A")

def main():
    rospy.init_node('byte_order_analyzer')
    
    def callback(msg):
        analyze_byte_order(msg)
    
    sub = rospy.Subscriber('/weld_state', WeldState, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
```

## 3. End-to-End Testing

### 3.1 Complete System Test

#### A. Test Sequence
```bash
# 1. Start robot-side logging
# On robot teach pendant, run io_test_weld program

# 2. Start ROS side with debug
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false _log_level:=debug

# 3. Monitor messages
rostopic echo /weld_state

# 4. Run validation
rosrun fanuc_driver weld_state_validator.py

# 5. Test with actual welding
# Start welding process and observe parameter changes
```

#### B. Create Test Harness
```python
#!/usr/bin/env python3
"""
Comprehensive test harness for welding state interface
"""
import rospy
from fanuc_driver.msg import WeldState
import time
import sys

class WeldStateTestHarness:
    def __init__(self):
        self.tests_passed = 0
        self.tests_failed = 0
        self.last_msg = None
        
    def test_message_reception(self):
        """Test that messages are being received"""
        rospy.loginfo("Testing message reception...")
        
        start_time = time.time()
        while time.time() - start_time < 5.0:  # Wait 5 seconds
            if self.last_msg is not None:
                rospy.loginfo("✓ Messages are being received")
                self.tests_passed += 1
                return True
            time.sleep(0.1)
            
        rospy.logerr("✗ No messages received in 5 seconds")
        self.tests_failed += 1
        return False
        
    def test_message_frequency(self):
        """Test message frequency"""
        rospy.loginfo("Testing message frequency...")
        
        msg_times = []
        start_time = time.time()
        
        while time.time() - start_time < 10.0:  # Collect for 10 seconds
            if self.last_msg is not None:
                msg_times.append(time.time())
                self.last_msg = None  # Reset for next message
            time.sleep(0.01)
            
        if len(msg_times) > 10:
            intervals = [msg_times[i] - msg_times[i-1] for i in range(1, len(msg_times))]
            avg_interval = sum(intervals) / len(intervals)
            frequency = 1.0 / avg_interval
            
            rospy.loginfo(f"Average frequency: {frequency:.1f} Hz")
            
            if 5.0 <= frequency <= 15.0:  # Expect ~10Hz
                rospy.loginfo("✓ Message frequency is reasonable")
                self.tests_passed += 1
                return True
            else:
                rospy.logwarn(f"✗ Message frequency unusual: {frequency:.1f} Hz")
                self.tests_failed += 1
                return False
        else:
            rospy.logerr("✗ Insufficient messages for frequency test")
            self.tests_failed += 1
            return False
            
    def message_callback(self, msg):
        self.last_msg = msg
        
    def run_tests(self):
        """Run all tests"""
        rospy.init_node('weld_state_test_harness')
        
        sub = rospy.Subscriber('/weld_state', WeldState, self.message_callback)
        
        rospy.loginfo("Starting welding state interface tests...")
        
        # Run tests
        self.test_message_reception()
        self.test_message_frequency()
        
        # Print results
        total_tests = self.tests_passed + self.tests_failed
        rospy.loginfo(f"Test Results: {self.tests_passed}/{total_tests} passed")
        
        if self.tests_failed == 0:
            rospy.loginfo("✓ All tests passed!")
            return 0
        else:
            rospy.logerr(f"✗ {self.tests_failed} tests failed")
            return 1

if __name__ == '__main__':
    harness = WeldStateTestHarness()
    result = harness.run_tests()
    sys.exit(result)
```

## 4. Common Issues and Solutions

### 4.1 KAREL Side Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| Registers not updating | R[10-14] values are 0 or constant | Check fieldbus I/O mapping configuration |
| TCP connection fails | "TCP_STAT returns 0" | Verify network settings, check firewall |
| Serialization errors | Garbage data in messages | Verify data types and byte alignment |

### 4.2 ROS Side Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| No messages received | Empty /weld_state topic | Check robot IP, verify KAREL program running |
| Wrong byte order | Unrealistic voltage/current values | Try use_bswap:=true in launch file |
| Connection timeouts | Intermittent connection errors | Check network stability, robot load |

### 4.3 Integration Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| Message type mismatch | ROS receives wrong message type | Verify RI_MT_WELDST = 15 in both sides |
| Data corruption | Random spikes in values | Check network quality, verify message size |
| Timing issues | Irregular message intervals | Check robot cycle time, network latency |

## 5. Debug Checklist

### Before Starting
- [ ] EWM welding power source connected and configured
- [ ] Fieldbus I/O mapping verified in robot controller
- [ ] KAREL programs compiled and loaded
- [ ] Network connectivity between robot and ROS PC confirmed

### KAREL Side
- [ ] Register values R[10-14] updating with welding parameters
- [ ] TCP connection established (TCP_STAT returns 1)
- [ ] Debug messages appearing in robot log
- [ ] Message serialization working correctly

### ROS Side
- [ ] TCP connection to robot successful
- [ ] Messages received on /weld_state topic
- [ ] Message values within reasonable ranges
- [ ] Correct byte order confirmed
- [ ] Message frequency stable (~10Hz)

### Integration
- [ ] End-to-end data flow verified
- [ ] Scaling factors produce correct engineering units
- [ ] Error conditions properly detected and reported
- [ ] System stable under continuous operation

Use this guide systematically to identify and resolve issues at each level of the welding state interface. 