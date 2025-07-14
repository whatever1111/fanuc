/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, YOUR_NAME
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the YOUR_NAME nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: YOUR_NAME
 */

#include <ros/ros.h>
#include <fanuc_driver/WeldState.h>
#include <simple_message/socket/tcp_client.h>
#include <simple_message/simple_message.h>
#include <simple_message/byte_array.h>
#include <industrial_utils/param_utils.h>

using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace industrial::byte_array;

// Message type constant matching KAREL RI_MT_WELDST
const int WELD_STATE_MSG_TYPE = 15;

// Scaling factors from EWM manual
const double VOLTAGE_SCALE = 100.0 / 32767.0;   // Raw to Volts
const double CURRENT_SCALE = 1000.0 / 32767.0;  // Raw to Amperes
const double WIRE_SPEED_SCALE = 40.0 / 32767.0; // Raw to m/min

class FanucWeldStateNode
{
private:
  ros::NodeHandle nh_;
  ros::Publisher weld_state_pub_;
  TcpClient tcp_client_;
  std::string robot_ip_;
  int robot_port_;
  
public:
  FanucWeldStateNode() : nh_("~")
  {
    // Get parameters
    nh_.param<std::string>("robot_ip", robot_ip_, "192.168.1.31");
    nh_.param<int>("robot_port", robot_port_, 11002);
    
    // Setup publisher
    weld_state_pub_ = nh_.advertise<fanuc_driver::WeldState>("weld_state", 10);
    
    ROS_INFO("Fanuc Weld State Node starting...");
    ROS_INFO("Connecting to robot at %s:%d", robot_ip_.c_str(), robot_port_);
  }
  
  bool init()
  {
    // Initialize TCP connection to robot
    if (!tcp_client_.init(const_cast<char*>(robot_ip_.c_str()), robot_port_))
    {
      ROS_ERROR("Failed to initialize TCP connection to robot");
      return false;
    }
    
    ROS_INFO("Successfully connected to robot");
    return true;
  }
  
  void run()
  {
    SimpleMessage msg;
    
    while (ros::ok())
    {
      // Receive message from robot
      if (tcp_client_.receiveMsg(msg))
      {
        // Check if this is a welding state message
        if (msg.getMessageType() == WELD_STATE_MSG_TYPE)
        {
          processWeldStateMessage(msg);
        }
        // Ignore other message types (joint states, robot status, etc.)
      }
      else
      {
        ROS_WARN_THROTTLE(5.0, "Failed to receive message from robot");
      }
      
      ros::spinOnce();
    }
  }
  
private:
  void processWeldStateMessage(const SimpleMessage& msg)
  {
    ByteArray data = msg.getData();
    
    // The message format matches our KAREL iwd_srlise function:
    // 4 booleans (as 4-byte integers) + 4 integers (4 bytes each) = 32 bytes
    if (data.getBufferSize() < 32)
    {
      ROS_WARN("Received welding state message with insufficient data size: %zu bytes", 
               data.getBufferSize());
      return;
    }
    
    fanuc_driver::WeldState weld_msg;
    
    // Parse the data (assuming little-endian format)
    int32_t arc_ok_int, ready_int, stick_err_int, general_err_int;
    int32_t err_code, act_voltage, act_current, act_wire_spd;
    
    // Unpack the data from the byte array
    data.unload(arc_ok_int);
    data.unload(ready_int);
    data.unload(stick_err_int);
    data.unload(general_err_int);
    data.unload(err_code);
    data.unload(act_voltage);
    data.unload(act_current);
    data.unload(act_wire_spd);
    
    // Convert to ROS message
    weld_msg.arc_ok = (arc_ok_int != 0);
    weld_msg.ready = (ready_int != 0);
    weld_msg.stick_err = (stick_err_int != 0);
    weld_msg.general_err = (general_err_int != 0);
    weld_msg.err_code = static_cast<uint8_t>(err_code);
    weld_msg.act_voltage = static_cast<int16_t>(act_voltage);
    weld_msg.act_current = static_cast<int16_t>(act_current);
    weld_msg.act_wire_spd = static_cast<int16_t>(act_wire_spd);
    
    // Publish the message
    weld_state_pub_.publish(weld_msg);
    
    // Optional: Log scaled values for debugging
    ROS_DEBUG("Weld State - Arc: %s, Ready: %s, Voltage: %.1fV, Current: %.0fA, Wire: %.1fm/min",
              weld_msg.arc_ok ? "OK" : "NO",
              weld_msg.ready ? "YES" : "NO",
              weld_msg.act_voltage * VOLTAGE_SCALE,
              weld_msg.act_current * CURRENT_SCALE,
              weld_msg.act_wire_spd * WIRE_SPEED_SCALE);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fanuc_weld_state_node");
  
  FanucWeldStateNode node;
  
  if (!node.init())
  {
    ROS_ERROR("Failed to initialize Fanuc Weld State Node");
    return -1;
  }
  
  ROS_INFO("Fanuc Weld State Node initialized successfully");
  node.run();
  
  return 0;
} 