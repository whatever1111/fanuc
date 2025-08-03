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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <simple_message/simple_message.h>
#include <simple_message/byte_array.h>

using namespace industrial::simple_message;
using namespace industrial::byte_array;

// Message type constant matching KAREL RI_MT_WELDST
const int WELD_STATE_MSG_TYPE = 15;

// Scaling factors from EWM manual
const double VOLTAGE_SCALE = 100.0 / 32767.0;   // Raw to Volts
const double CURRENT_SCALE = 1000.0 / 32767.0;  // Raw to Amperes
const double WIRE_SPEED_SCALE = 40.0 / 32767.0;  // Raw to m/min

class FanucWeldStateNodeSimple
{
private:
  ros::NodeHandle nh_;
  ros::Publisher weld_state_pub_;
  std::string robot_ip_;
  int robot_port_;
  int sock_fd_;
  bool debug_;
  
public:
  FanucWeldStateNodeSimple() : nh_("~"), sock_fd_(-1)
  {
    // Get parameters
    nh_.param<std::string>("robot_ip", robot_ip_, "127.0.0.1");
    nh_.param<int>("robot_port", robot_port_, 11002);
    nh_.param<bool>("debug", debug_, false);
    
    // Setup publisher
    weld_state_pub_ = nh_.advertise<fanuc_driver::WeldState>("weld_state", 10);
    
    ROS_INFO("Fanuc Weld State Simple Message Node Starting");
    ROS_INFO("Target: %s:%d", robot_ip_.c_str(), robot_port_);
    if (debug_) ROS_INFO("Debug mode enabled");
  }
  
  ~FanucWeldStateNodeSimple()
  {
    if (sock_fd_ >= 0)
    {
      close(sock_fd_);
    }
  }

  bool init()
  {
    ROS_INFO("Connecting to KAREL program...");
    
    // Create socket
    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0)
    {
      ROS_ERROR("Failed to create socket");
      return false;
    }
    
    // Setup server address
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(robot_port_));
    
    if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0)
    {
      ROS_ERROR("Invalid IP address: %s", robot_ip_.c_str());
      close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    
    // Connect to server
    if (connect(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
      ROS_ERROR("Failed to connect to %s:%d", robot_ip_.c_str(), robot_port_);
      ROS_ERROR("Make sure KAREL program is running and cfg_.checked = TRUE");
      close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    
    ROS_INFO("Successfully connected to robot via TCP");
    return true;
  }
  
  void run()
  {
    int msg_count = 0;
    int error_count = 0;
    
    ROS_INFO("Starting weld state monitoring...");
    
    while (ros::ok())
    {
      // Receive Standard Simple Message format
      SimpleMessage msg;
      if (receiveSimpleMessage(msg))
      {
        msg_count++;
        if (debug_)
        {
          ROS_INFO("Message #%d - Type: %d, Length: %d", 
                   msg_count, msg.getMessageType(), msg.getData().getBufferSize());
        }
        
        // Check if this is a welding state message
        if (msg.getMessageType() == WELD_STATE_MSG_TYPE)
        {
          processWeldStateMessage(msg);
          error_count = 0;  // Reset error count on success
        }
        else
        {
          if (debug_) ROS_DEBUG("Ignoring message type: %d", msg.getMessageType());
        }
      }
      else
      {
        error_count++;
        if (error_count % 10 == 1)
        {
          ROS_WARN("Communication error (count: %d)", error_count);
        }
        
        // Try to reconnect
        if (error_count > 5 && !reconnect())
        {
          ROS_ERROR("Failed to reconnect, exiting");
          break;
        }
      }
      
      ros::spinOnce();
    }
  }
  
private:
  bool receiveExactly(uint8_t* buffer, size_t bytes)
  {
    size_t received = 0;
    while (received < bytes)
    {
      ssize_t result = recv(sock_fd_, buffer + received, bytes - received, 0);
      if (result <= 0)
      {
        if (result == 0)
        {
          ROS_WARN("Connection closed by server");
        }
        else
        {
          ROS_WARN("recv() error: %s", strerror(errno));
        }
        return false;
      }
      received += static_cast<size_t>(result);
    }
    return true;
  }
  
  bool receiveSimpleMessage(SimpleMessage& msg)
  {
    // Receive header (16 bytes: length, msg_type, comm_type, reply_type)
    uint8_t header_buf[16];
    if (!receiveExactly(header_buf, 16))
    {
      return false;
    }
    
    // Parse header (Data from KAREL is Little Endian)
    uint32_t length = *((uint32_t*)&header_buf[0]);
    uint32_t msg_type = *((uint32_t*)&header_buf[4]);
    uint32_t comm_type = *((uint32_t*)&header_buf[8]);
    uint32_t reply_type = *((uint32_t*)&header_buf[12]);
    
    if (debug_)
    {
      ROS_DEBUG("Header - Length: %u, Type: %u, Comm: %u, Reply: %u", 
                length, msg_type, comm_type, reply_type);
    }
    
    // Handle different message formats
    uint8_t* payload_buf = nullptr;
    
    if (length == 32 && msg_type == 15 && comm_type == 1)
    {
      // Format 1: Only payload (32 bytes), no sequence number
      if (debug_) ROS_DEBUG("Receiving payload-only format (length=32)");
      
      uint8_t* direct_payload = new uint8_t[32];
      if (!receiveExactly(direct_payload, 32))
      {
        delete[] direct_payload;
        return false;
      }
      payload_buf = direct_payload;
      
      // Peek to check whether a single terminator byte (CR or LF) follows the
      // payload. Only consume it if it really is a terminator; otherwise leave
      // the byte in the stream so that the next header is not corrupted.
      uint8_t peek_byte;
      ssize_t peek_ret = recv(sock_fd_, &peek_byte, 1, MSG_PEEK | MSG_DONTWAIT);
      if (peek_ret == 1 && (peek_byte == '\r' || peek_byte == '\n'))
      {
        recv(sock_fd_, &peek_byte, 1, 0);  // remove the byte we just peeked
        if (debug_) ROS_DEBUG("Discarded terminator byte: 0x%02x", peek_byte);
      }
    }
    else if (length == 53 && msg_type == 15 && comm_type == 1)
    {
      // Format 2: Standard Simple Message format with sequence number
      if (debug_) ROS_DEBUG("Receiving standard format (length=53)");
      
      // Receive remaining 40 bytes (4 bytes sequence + 36 bytes payload)
      uint8_t remaining_buf[40];
      if (!receiveExactly(remaining_buf, 40))
      {
        return false;
      }
      
      // Parse payload (skip sequence number, use remaining 36 bytes)
      payload_buf = &remaining_buf[4];
      
      // Optionally remove a single terminator byte (CR or LF) that may follow
      // the payload. Use the same peek-and-consume strategy as above to avoid
      // accidentally dropping the first byte of the next message.
      uint8_t peek_byte2;
      ssize_t peek_ret2 = recv(sock_fd_, &peek_byte2, 1, MSG_PEEK | MSG_DONTWAIT);
      if (peek_ret2 == 1 && (peek_byte2 == '\r' || peek_byte2 == '\n'))
      {
        recv(sock_fd_, &peek_byte2, 1, 0);
        if (debug_) ROS_DEBUG("Discarded terminator byte after standard format: 0x%02x", peek_byte2);
      }
    }
    else
    {
      ROS_WARN("Invalid header - Expected: (Length=32 or 53), Type=15, Comm=1");
      if (debug_)
      {
        ROS_WARN("Received: Length=%u, Type=%u, Comm=%u, Reply=%u", length, msg_type, comm_type, reply_type);
      }
      return false;
    }
    
    // Create Simple Message object
    ByteArray data;
    
    // Load data in normal order for FIFO unload
    for (int i = 0; i < 9; i++)
    {
      int32_t value = *((int32_t*)&payload_buf[i * 4]);
      data.load(value);
    }
    
    // Explicitly cast to signed 32-bit integers to avoid -Wsign-conversion warnings
    msg.init(static_cast<int32_t>(msg_type),
             static_cast<int32_t>(comm_type),
             static_cast<int32_t>(reply_type),
             data);
    
    // Clean up if we allocated memory
    if (length == 32)
    {
      delete[] payload_buf;
    }
    
    return true;
  }
  
  bool reconnect()
  {
    ROS_INFO("Attempting to reconnect...");
    
    if (sock_fd_ >= 0)
    {
      close(sock_fd_);
      sock_fd_ = -1;
    }
    
    sleep(1); // Wait before reconnecting
    
    return init();
  }

  void processWeldStateMessage(const SimpleMessage& msg)
  {
    // Get the raw data buffer
    ByteArray data = const_cast<SimpleMessage&>(msg).getData();
    
    if (data.getBufferSize() < 36)
    {
      ROS_WARN("Insufficient data size: %u bytes", data.getBufferSize());
      return;
    }
    
    // Get the raw buffer
    std::vector<char> buffer;
    data.copyTo(buffer);
    
    if (buffer.size() < 36)
    {
      ROS_ERROR("Buffer size too small: %lu bytes", buffer.size());
      return;
    }
    
    // Parse weld data (Little Endian)
    int32_t weld_ints[9];
    for (std::size_t i = 0; i < 9; i++)
    {
      weld_ints[i] = *reinterpret_cast<int32_t*>(&buffer[i * 4]);
    }
    
    if (debug_)
    {
      ROS_INFO("Raw weld data:");
      ROS_INFO("  arc_ok: %d, power_err: %d, depos_di: %d", 
               weld_ints[0], weld_ints[1], weld_ints[2]);
      ROS_INFO("  ewm_err: %d, warn: %d, voltage: %d, current: %d, wire_spd: %d, motor_curr: %d",
               weld_ints[3], weld_ints[4], weld_ints[5], weld_ints[6], weld_ints[7], weld_ints[8]);
    }
    
    fanuc_driver::WeldState weld_msg;
    
    // Convert to ROS message
    weld_msg.arc_ok        = (weld_ints[0] != 0);
    
    weld_msg.power_err     = (weld_ints[1] != 0);
    weld_msg.depos_di      = (weld_ints[2] != 0);
    weld_msg.ewm_err       = static_cast<int16_t>(weld_ints[3]);
    weld_msg.warning_state = static_cast<int16_t>(weld_ints[4]);
    weld_msg.act_voltage   = static_cast<int16_t>(weld_ints[5]);
    weld_msg.act_current   = static_cast<int16_t>(weld_ints[6]);
    weld_msg.act_wire_spd  = static_cast<int16_t>(weld_ints[7]);
    weld_msg.motor_current = static_cast<int16_t>(weld_ints[8]);
    
    if (debug_)
    {
      ROS_INFO("Welding State: arc_ok=%s, power_err=%s, voltage=%.1fV, current=%.0fA",
               weld_msg.arc_ok ? "TRUE" : "FALSE",
               weld_msg.power_err ? "TRUE" : "FALSE",
               weld_msg.act_voltage * VOLTAGE_SCALE,
               weld_msg.act_current * CURRENT_SCALE);
    }
    
    // Publish the message
    weld_state_pub_.publish(weld_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fanuc_weld_state_node_simple");
  
  FanucWeldStateNodeSimple node;
  
  if (!node.init())
  {
    ROS_ERROR("Failed to initialize Fanuc Weld State Simple Message Node");
    return -1;
  }
  
  ROS_INFO("Fanuc Weld State Simple Message Node initialized successfully");
  node.run();
  
  return 0;
} 