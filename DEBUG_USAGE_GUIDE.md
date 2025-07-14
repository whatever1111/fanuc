# Fanuc Welding State Debug Tools - Usage Guide

This guide provides step-by-step instructions for using the debug tools to troubleshoot the welding state interface.

## Quick Start

### 1. Test All Debug Tools
```bash
# First, verify all debug tools are working
cd ~/catkin_ws/src/fanuc/fanuc_driver
./scripts/test_debug_tools.py
```

### 2. Basic Connectivity Test
```bash
# Test basic network connectivity to robot
ping <ROBOT_IP>

# Test if TCP port is accessible
telnet <ROBOT_IP> 11002
```

## Debug Workflow

### Phase 1: KAREL Side Debugging

#### Step 1: Verify KAREL Programs are Loaded
1. On the robot teach pendant, navigate to: `Menu → SETUP → Host Comm → Programs`
2. Verify these programs are present:
   - `ros_state.kl`
   - `libind_weld_t.kl`
   - `libind_weld_h.kl`

#### Step 2: Check Register Values
1. On teach pendant: `Menu → I/O → Register`
2. Check registers R[10], R[12], R[13], R[14]
3. Values should change when welding parameters change

#### Step 3: Run KAREL Debug Program
```karel
-- Load and run the debug program on robot controller
PROGRAM weld_io_test
-- This will continuously monitor and display register values
```

Expected output:
```
========================================
WELDING I/O DEBUG - Cycle 1
========================================
Status Word R[10] = 3
Voltage R[12] = 8192
Current R[13] = 16384
Wire Speed R[14] = 4096
----------------------------------------
STATUS WORD BIT ANALYSIS:
  Bit 0 (arc_ok): TRUE
  Bit 1 (ready): TRUE
  Bit 2 (stick_err): FALSE
  Bit 3 (general_err): FALSE
  Error Code (bits 8-15): 0
```

### Phase 2: Network Communication Testing

#### Step 1: Capture Raw Messages
```bash
# Capture raw TCP messages from robot
./scripts/raw_message_capture.py <ROBOT_IP>
```

Expected output:
```
Connected to 192.168.1.31:11002
--- Message 1 ---
Header: Length=32, Type=15
Data (32 bytes): 01000000010000000000000000000000...
>>> WELDING STATE MESSAGE <<<
Parsed values:
  arc_ok = 1 (TRUE)
  ready = 1 (TRUE)
  stick_err = 0 (FALSE)
  general_err = 0 (FALSE)
  err_code = 0
  act_voltage = 8192 (raw)
  act_current = 16384 (raw)
  act_wire_spd = 4096 (raw)
Scaled values:
  Voltage: 25.0V
  Current: 500A
  Wire Speed: 5.0m/min
```

#### Step 2: Test Different Byte Orders
```bash
# Test normal byte order
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false

# In another terminal, analyze byte order
./scripts/byte_order_analyzer.py
```

### Phase 3: ROS Side Debugging

#### Step 1: Start ROS Node with Debug Logging
```bash
# Start with debug logging enabled
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false _log_level:=debug
```

#### Step 2: Validate Messages
```bash
# In another terminal, run message validator
./scripts/weld_state_validator.py
```

Expected output:
```
============================================================
VALIDATION STATISTICS (Runtime: 10.0s)
Messages: 100 (10.0 Hz)
Errors: 0 (0.0%)
Warnings: 2 (2.0%)
Max consecutive errors: 0
Message intervals: avg=0.100s, min=0.095s, max=0.105s
Voltage range: 20.0V to 30.0V (avg: 25.0V)
Current range: 200A to 600A (avg: 400A)
============================================================
```

#### Step 3: Monitor Welding State
```bash
# Start the monitoring script
./scripts/weld_state_monitor.py
```

## Troubleshooting Common Issues

### Issue 1: No Messages Received

**Symptoms:**
- `rostopic echo /weld_state` shows no output
- Raw message capture shows "Connection refused"

**Debug Steps:**
1. Check robot connectivity:
   ```bash
   ping <ROBOT_IP>
   nmap -p 11002 <ROBOT_IP>
   ```

2. Verify KAREL program is running:
   ```bash
   # Check if port is listening
   telnet <ROBOT_IP> 11002
   ```

3. Check robot controller logs for errors

**Solutions:**
- Restart `ros_state.kl` program on robot
- Check network configuration
- Verify firewall settings

### Issue 2: Incorrect Values

**Symptoms:**
- Voltage shows as 32000V instead of 25V
- Current shows negative values
- Wire speed is unrealistic

**Debug Steps:**
1. Run byte order analyzer:
   ```bash
   ./scripts/byte_order_analyzer.py
   ```

2. Check raw message data:
   ```bash
   ./scripts/raw_message_capture.py <ROBOT_IP>
   ```

**Solutions:**
- Try opposite byte order: `use_bswap:=true`
- Verify register mapping in KAREL code
- Check EWM fieldbus configuration

### Issue 3: Intermittent Connection

**Symptoms:**
- Messages stop occasionally
- "Message gap too large" warnings
- Connection timeouts

**Debug Steps:**
1. Monitor network traffic:
   ```bash
   sudo tcpdump -i eth0 host <ROBOT_IP> and port 11002
   ```

2. Check message timing:
   ```bash
   ./scripts/weld_state_validator.py
   ```

**Solutions:**
- Check network cable connections
- Reduce robot controller load
- Adjust TCP timeout settings

### Issue 4: Wrong Message Type

**Symptoms:**
- ROS node receives messages but ignores them
- Raw capture shows different message type

**Debug Steps:**
1. Check message type in raw capture
2. Verify `RI_MT_WELDST = 15` in KAREL code

**Solutions:**
- Update message type constant in KAREL
- Verify message type in ROS node

## Step-by-Step Debugging Procedure

### When Starting from Scratch

1. **Verify Hardware Setup**
   - EWM welding power source connected to robot via fieldbus
   - Robot controller has fieldbus I/O module
   - Network connectivity between robot and ROS PC

2. **Test Basic Connectivity**
   ```bash
   ping <ROBOT_IP>
   telnet <ROBOT_IP> 11002
   ```

3. **Check KAREL Side**
   ```bash
   # Run KAREL debug program on robot
   # Monitor register values R[10-14]
   # Verify values change during welding
   ```

4. **Test Raw Communication**
   ```bash
   ./scripts/raw_message_capture.py <ROBOT_IP>
   ```

5. **Start ROS Node**
   ```bash
   roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false
   ```

6. **Validate Messages**
   ```bash
   ./scripts/weld_state_validator.py
   ```

7. **Monitor Real-time Data**
   ```bash
   ./scripts/weld_state_monitor.py
   ```

### When Debugging Existing Setup

1. **Check Current Status**
   ```bash
   rostopic echo /weld_state
   ```

2. **Run Validator**
   ```bash
   ./scripts/weld_state_validator.py
   ```

3. **If Issues Found, Capture Raw Data**
   ```bash
   ./scripts/raw_message_capture.py <ROBOT_IP>
   ```

4. **Check Byte Order**
   ```bash
   ./scripts/byte_order_analyzer.py
   ```

5. **Verify KAREL Side**
   ```bash
   # Run weld_io_test.kl on robot
   # Check register values
   ```

## Debug Tool Reference

### Raw Message Capture
```bash
./scripts/raw_message_capture.py <ROBOT_IP> [--port 11002] [--timeout 30]
```
- Captures raw TCP messages from robot
- Shows message headers and data
- Parses welding state messages
- Applies scaling factors

### Message Validator
```bash
./scripts/weld_state_validator.py
```
- Validates message timing and content
- Checks for reasonable value ranges
- Detects stuck or corrupted data
- Provides statistics and error rates

### Byte Order Analyzer
```bash
./scripts/byte_order_analyzer.py
```
- Analyzes whether byte swapping is needed
- Compares normal vs swapped interpretations
- Scores values based on welding reasonableness
- Provides recommendations

### Welding State Monitor
```bash
./scripts/weld_state_monitor.py
```
- Real-time display of welding parameters
- Applies proper scaling factors
- Shows warnings for error conditions
- User-friendly output format

### Test Suite
```bash
./scripts/test_debug_tools.py
```
- Verifies all debug tools are working
- Tests script permissions and imports
- Checks KAREL file existence
- Provides overall system health check

## Log Analysis

### KAREL Side Logs
Look for these patterns in robot controller logs:
```
WELD_DEBUG: Status=3, V=8192, I=16384, W=4096
WELD_MSG_SENT: OK
TCP_STATUS: 1
```

### ROS Side Logs
Look for these patterns in ROS logs:
```
[DEBUG] Received message - Type: 15, Size: 32
[DEBUG] Parsed values - arc_ok: 1, ready: 1, voltage: 8192, current: 16384
[INFO] Weld Status: ARC ON | READY | V: 25.0V | I: 500A | Wire: 5.0m/min
```

## Performance Monitoring

### Expected Performance
- Message frequency: 8-12 Hz
- Message size: 32 bytes
- Network latency: < 10ms
- Error rate: < 1%

### Warning Signs
- Message frequency < 5 Hz or > 20 Hz
- Large message gaps (> 200ms)
- High error rates (> 5%)
- Stuck or constant values

Use this guide systematically to identify and resolve issues with the welding state interface. 