# Fanuc Welding State Interface Setup Guide

This guide explains how to configure and use the welding state interface for Fanuc robots connected to EWM welding power sources via fieldbus communication.

## Overview

The welding state interface allows ROS to receive real-time welding parameters from an EWM welding power source (Phoenix, alpha Q series) that is connected to the Fanuc robot controller via Profinet or Ethernet/IP fieldbus.

### System Architecture

```
EWM Welding Power Source (Phoenix/alpha Q)
    ↓ (Profinet/EthernetIP)
Fanuc Robot Controller (R-30iA/B)
    ↓ (TCP/IP Port 11002)
ROS PC running fanuc_driver
```

## Hardware Requirements

1. **EWM Welding Power Source**: Phoenix or alpha Q series with BUSINT X11 fieldbus interface
2. **Fanuc Robot Controller**: R-30iA or R-30iB with fieldbus capability
3. **Network Infrastructure**: Ethernet network connecting welding power source, robot controller, and ROS PC

## Setup Instructions

### 1. EWM Welding Power Source Configuration

Configure the EWM welding power source according to the `099-008225-EWM21` manual:

- Set up the fieldbus interface (Profinet or Ethernet/IP)
- Configure the welding parameters profile for Multimatrix series
- Ensure the welding power source is properly connected to the robot controller's fieldbus network

### 2. Fanuc Robot Controller Configuration

#### A. Fieldbus I/O Mapping

Configure the robot controller to map the EWM fieldbus data to internal registers:

1. **Status Word Mapping**: Map EWM Input Word 0 (containing status bits and error code) to a Fanuc register, e.g., `R[10]`
2. **Analog Value Mapping**: Map the following EWM input words to consecutive Fanuc registers:
   - EWM Input Word 3 (GI_RealVoltage) → `R[12]`
   - EWM Input Word 4 (GI_RealCurrent) → `R[13]`
   - EWM Input Word 5 (GI_RealWireSpeed) → `R[14]`

#### B. Update KAREL Constants

Edit `fanuc_driver/karel/include/libind_weld_t.kl` and update the register mapping constants to match your configuration:

```karel
CONST
    WELD_R_STATUS_WORD = 10  -- Your chosen register for EWM status word
    WELD_R_ACT_VOLT    = 12  -- Your chosen register for voltage
    WELD_R_ACT_CURR    = 13  -- Your chosen register for current
    WELD_R_ACT_WIRE    = 14  -- Your chosen register for wire speed
```

#### C. Compile and Load KAREL Programs

1. Compile the modified KAREL programs using Roboguide or the robot controller
2. Load the following programs to the robot controller:
   - `ros_state.kl` (modified to include welding state)
   - `libind_weld_t.kl` (welding data types)
   - `libind_weld_h.kl` (welding function prototypes)

### 3. ROS Side Configuration

#### A. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### B. Launch the Welding State Node

```bash
# For standard byte order (most common)
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=false

# For byte-swapped systems (if required)
roslaunch fanuc_driver weld_state.launch robot_ip:=<ROBOT_IP> use_bswap:=true
```

Replace `<ROBOT_IP>` with your robot controller's IP address.

## Usage

### Monitoring Welding State

The welding state is published on the `/weld_state` topic as `fanuc_driver/WeldState` messages.

#### Using the Python Monitor Script

```bash
rosrun fanuc_driver weld_state_monitor.py
```

This will display real-time welding parameters:
```
[INFO] Weld Status: ARC ON | READY | V: 28.5V | I: 245A | Wire: 8.2m/min
```

#### Subscribing in Your Own Code

**Python Example:**
```python
import rospy
from fanuc_driver.msg import WeldState

def weld_callback(msg):
    # Scale raw values to engineering units
    voltage = msg.act_voltage * 100.0 / 32767.0      # Volts
    current = msg.act_current * 1000.0 / 32767.0     # Amperes
    wire_speed = msg.act_wire_spd * 40.0 / 32767.0   # m/min
    
    print(f"Arc: {msg.arc_ok}, Voltage: {voltage:.1f}V, Current: {current:.0f}A")

rospy.init_node('my_weld_monitor')
sub = rospy.Subscriber('/weld_state', WeldState, weld_callback)
rospy.spin()
```

**C++ Example:**
```cpp
#include <ros/ros.h>
#include <fanuc_driver/WeldState.h>

void weldCallback(const fanuc_driver::WeldState::ConstPtr& msg)
{
    // Scale raw values to engineering units
    double voltage = msg->act_voltage * 100.0 / 32767.0;      // Volts
    double current = msg->act_current * 1000.0 / 32767.0;     // Amperes
    double wire_speed = msg->act_wire_spd * 40.0 / 32767.0;   // m/min
    
    ROS_INFO("Arc: %s, Voltage: %.1fV, Current: %.0fA", 
             msg->arc_ok ? "ON" : "OFF", voltage, current);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_weld_monitor");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/weld_state", 10, weldCallback);
    ros::spin();
    
    return 0;
}
```

## Message Format

The `fanuc_driver/WeldState` message contains:

```
bool arc_ok         # I>0 signal - Arc is established and stable
bool ready          # Welder is ready for operation
bool stick_err      # Wire stick error detected
bool general_err    # General error condition active
uint8 err_code      # Specific fault code from welding power source
int16 act_voltage   # Raw actual welding voltage (scale: × 100.0/32767.0)
int16 act_current   # Raw actual welding current (scale: × 1000.0/32767.0)
int16 act_wire_spd  # Raw actual wire speed (scale: × 40.0/32767.0)
```

## Troubleshooting

### Common Issues

1. **No welding data received**
   - Check fieldbus connection between EWM and robot controller
   - Verify I/O mapping configuration in robot controller
   - Ensure KAREL programs are loaded and running

2. **Incorrect scaling**
   - Verify the scaling factors match your EWM power source specifications
   - Check the EWM manual for your specific model

3. **Connection errors**
   - Verify robot IP address and network connectivity
   - Check that port 11002 is not blocked by firewalls
   - Ensure the `ros_state.kl` program is running on the robot

### Debug Information

Enable debug logging to see raw message data:
```bash
rosrun fanuc_driver weld_state _log_level:=debug
```

## Integration with MoveIt and Other ROS Packages

The welding state can be integrated into your robotic welding applications:

1. **Weld Quality Monitoring**: Monitor arc stability and current/voltage for quality control
2. **Adaptive Welding**: Adjust robot speed based on welding parameters
3. **Error Handling**: Automatically stop or adjust welding when errors are detected
4. **Data Logging**: Record welding parameters for process documentation

## References

- EWM BUSINT X11 Manual: `099-008225-EWM21`
- Fanuc KAREL Programming Manual
- ROS-Industrial Simple Message Protocol Documentation 