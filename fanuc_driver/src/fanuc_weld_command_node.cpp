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
 */

#include <cstdint>
#include <ros/ros.h>
#include <fanuc_driver/WeldCommand.h>
#include <std_msgs/Header.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <algorithm>
#include <cctype>

class FanucWeldCommandNode
{
private:
    ros::NodeHandle nh_; // Private nodehandle for parameters
    ros::NodeHandle public_nh_; // Public nodehandle for topics
    ros::Subscriber command_sub_;
    
    std::string robot_ip_;
    int robot_port_;
    int sock_fd_;
    bool connected_;
    bool little_endian_;  // true: little-endian (default), false: big-endian
    bool heartbeat_enabled_; // default false: disable heartbeat unless requested
    std::mutex socket_mutex_;
    
    // ROS-Industrial packet structure for welding commands
    struct WeldCommandPacket {
        // Header (16 bytes) - matches ind_hdr_t in KAREL
        uint32_t length;
        uint32_t msg_type;
        uint32_t comm_type;
        uint32_t reply_type;
        
        // Sequence number (4 bytes)
        uint32_t seq_nr;
        
        // Payload (40 bytes = 10 * 4 bytes)
        uint32_t target_wire_spd;
        uint32_t correction_val;
        uint32_t dyn_setting;
        uint32_t operation_mode;
        uint32_t std_pulse_val;
        uint32_t program_number;
        uint32_t arc_start_cmd;
        uint32_t gas_control;
        uint32_t jog_feed_cmd;
        uint32_t jog_retract_cmd;
    } __attribute__((packed));
    
    static const uint32_t RI_MT_WELDCMD = 14;  // Weld Command message type
    static const uint32_t RI_CT_SVCREQ = 1;   // Service Request
    static const uint32_t RI_RT_INVAL = 0;    // Invalid reply type for requests
    static const uint32_t NO_CHANGE = 0xFFFFFFFF;  // Sentinel value for unchanged parameters (all bits set)
    static const int32_t RI_SEQ_HB = -110;    // Heartbeat frame sequence number
    
    uint32_t sequence_number_;
    ros::Timer heartbeat_timer_;

public:
    FanucWeldCommandNode() : nh_("~"), sock_fd_(-1), connected_(false), sequence_number_(0)
    {
        // Get parameters from private namespace
        nh_.param<std::string>("robot_ip", robot_ip_, "127.0.0.1");
        nh_.param<int>("robot_port", robot_port_, 11002);

        // Byte order parameter: "little" (default) or "big"
        std::string byte_order_param;
        nh_.param<std::string>("byte_order", byte_order_param, std::string("little"));
        std::transform(byte_order_param.begin(), byte_order_param.end(), byte_order_param.begin(), ::tolower);
        little_endian_ = (byte_order_param == "little" || byte_order_param == "le");
        ROS_INFO("Expecting %s-endian robot messages", little_endian_ ? "little" : "big");

        // Heartbeat enable parameter (default false)
        nh_.param<bool>("enable_heartbeat", heartbeat_enabled_, false);
        if (heartbeat_enabled_) {
            heartbeat_timer_ = public_nh_.createTimer(ros::Duration(3.0),
                                             &FanucWeldCommandNode::heartbeatCallback, this);
            ROS_INFO("Heartbeat enabled (3s)");
        } else {
            ROS_INFO("Heartbeat disabled");
        }
        
        ROS_INFO("Fanuc Weld Command Node starting");
        ROS_INFO("Target robot: %s:%d", robot_ip_.c_str(), robot_port_);
        
        // Subscribe to weld command topic using the public nodehandle
        command_sub_ = public_nh_.subscribe("/weld_command", 10, 
                                   &FanucWeldCommandNode::commandCallback, this);
        

        ROS_INFO("Subscribed to /weld_command topic");
        ROS_INFO("Ready to send welding commands to robot");
    }
    
    ~FanucWeldCommandNode()
    {
        disconnect();
    }
    
    bool connect()
    {
        if (connected_) {
            return true;
        }
        
        sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd_ < 0) {
            ROS_ERROR("Failed to create socket: %s", strerror(errno));
            return false;
        }
        
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        // Explicitly cast to uint16_t to avoid implicit narrowing warning
        server_addr.sin_port = htons(static_cast<uint16_t>(robot_port_));
        
        if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0) {
            ROS_ERROR("Invalid IP address: %s", robot_ip_.c_str());
            close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }
        
        if (::connect(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            ROS_WARN("Failed to connect to robot at %s:%d - %s", 
                     robot_ip_.c_str(), robot_port_, strerror(errno));
            close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }
        
        connected_ = true;
        ROS_INFO("Connected to robot at %s:%d", robot_ip_.c_str(), robot_port_);
        return true;
    }
    
    void disconnect()
    {
        if (sock_fd_ >= 0) {
            close(sock_fd_);
            sock_fd_ = -1;
        }
        connected_ = false;
    }
    
    void commandCallback(const fanuc_driver::WeldCommand::ConstPtr& msg)
    {
        ROS_INFO("--- Received WeldCommand ROS message ---");
        ROS_INFO("  prog_number: %d", (int32_t)msg->program_number);
        ROS_INFO("  target_wire_spd: %d", (int32_t)msg->target_wire_spd);
        ROS_INFO("  correction_val: %d", (int32_t)msg->correction_val);
        ROS_INFO("  dyn_setting: %d", (int32_t)msg->dyn_setting);
        ROS_INFO("  operation_mode: %d", (int32_t)msg->operation_mode);
        ROS_INFO("  std_pulse_val: %d", (int32_t)msg->std_pulse_val);
        ROS_INFO("  arc_start_cmd: %d", (int32_t)msg->arc_start_cmd);
        ROS_INFO("  gas_control: %d", (int32_t)msg->gas_control);
        ROS_INFO("  jog_feed_cmd: %d", (int32_t)msg->jog_feed_cmd);
        ROS_INFO("  jog_retract_cmd: %d", (int32_t)msg->jog_retract_cmd);
        
        std::lock_guard<std::mutex> lk(socket_mutex_);
        if (!connect()) {
            ROS_WARN("Cannot send command - not connected to robot");
            return;
        }
        
        // Prepare packet
        WeldCommandPacket packet;
        memset(&packet, 0, sizeof(packet));
        
        // Header
        packet.length = sizeof(packet);
        packet.msg_type = RI_MT_WELDCMD;
        packet.comm_type = RI_CT_SVCREQ;
        packet.reply_type = RI_RT_INVAL;
        
        // Sequence number
        packet.seq_nr = ++sequence_number_;
        
        // Copy values from ROS message.
        // The publisher of the message is responsible for setting unused fields to the
        // sentinel value NO_CHANGE (0xFFFFFFFF).
        packet.target_wire_spd = static_cast<uint32_t>(msg->target_wire_spd);
        packet.correction_val  = static_cast<uint32_t>(msg->correction_val);
        packet.dyn_setting     = static_cast<uint32_t>(msg->dyn_setting);
        packet.operation_mode  = static_cast<uint32_t>(msg->operation_mode);
        packet.std_pulse_val   = static_cast<uint32_t>(msg->std_pulse_val);
        packet.program_number  = static_cast<uint32_t>(msg->program_number);
        packet.arc_start_cmd   = static_cast<uint32_t>(msg->arc_start_cmd);
        packet.gas_control     = static_cast<uint32_t>(msg->gas_control);
        packet.jog_feed_cmd    = static_cast<uint32_t>(msg->jog_feed_cmd);
        packet.jog_retract_cmd = static_cast<uint32_t>(msg->jog_retract_cmd);

        // Build send buffer (convert to big-endian in a separate buffer to avoid
        // corrupting host-order fields that might be reused later).
        uint8_t send_buf[sizeof(packet)];
        std::memcpy(send_buf, &packet, sizeof(packet));
        if (!little_endian_)
        {
            uint32_t* p = reinterpret_cast<uint32_t*>(send_buf);
            size_t cnt = sizeof(packet) / sizeof(uint32_t);
            for (size_t i = 0; i < cnt; ++i)
            {
                p[i] = htonl(p[i]);
            }
        }
        
        ROS_INFO("--- Sending TCP Packet (seq: %u) ---", packet.seq_nr);
        ROS_INFO("  prog_number: %d", (int32_t)packet.program_number);
        ROS_INFO("  target_wire_spd: %d", (int32_t)packet.target_wire_spd);
        ROS_INFO("  correction_val: %d", (int32_t)packet.correction_val);
        ROS_INFO("  dyn_setting: %d", (int32_t)packet.dyn_setting);
        ROS_INFO("  operation_mode: %d", (int32_t)packet.operation_mode);
        ROS_INFO("  std_pulse_val: %d", (int32_t)packet.std_pulse_val);
        ROS_INFO("  arc_start_cmd: %d", (int32_t)packet.arc_start_cmd);
        ROS_INFO("  gas_control: %d", (int32_t)packet.gas_control);
        ROS_INFO("  jog_feed_cmd: %d", (int32_t)packet.jog_feed_cmd);
        ROS_INFO("  jog_retract_cmd: %d", (int32_t)packet.jog_retract_cmd);

        // Send packet
        ssize_t bytes_sent = send(sock_fd_, send_buf, sizeof(packet), 0);
        if (bytes_sent != sizeof(packet)) {
            ROS_ERROR("Failed to send complete packet: sent %zd of %zu bytes", 
                      bytes_sent, sizeof(packet));
            disconnect();
            return;
        }
        
        ROS_DEBUG("Sent weld command packet (seq: %u, %zu bytes)", 
                  packet.seq_nr, sizeof(packet));
        
        // Wait for reply (simple acknowledgment)
        struct {
            uint32_t length;
            uint32_t msg_type;
            uint32_t comm_type;
            uint32_t reply_type;
            uint32_t seq_nr;
        } reply;
        
        // Non-blocking read of acknowledgment (avoid blocking the callback)
        size_t expected_bytes = sizeof(reply);
        size_t received_total = 0;
        while (received_total < expected_bytes)
        {
            ssize_t r = recv(sock_fd_, ((char*)&reply) + received_total,
                             expected_bytes - received_total, MSG_DONTWAIT);
            if (r < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;  // No more data available right now
                ROS_WARN("recv() error while waiting for ACK: %s", strerror(errno));
                disconnect();
                break;
            }
            else if (r == 0)
            {
                ROS_WARN("Connection closed by robot while waiting for ACK");
                disconnect();
                break;
            }
            else
            {
                received_total += static_cast<size_t>(r);
            }
        }

        if (!little_endian_)
        {
            reply.length     = ntohl(reply.length);
            reply.msg_type   = ntohl(reply.msg_type);
            reply.comm_type  = ntohl(reply.comm_type);
            reply.reply_type = ntohl(reply.reply_type);
            reply.seq_nr     = ntohl(reply.seq_nr);
        }

        if (received_total == expected_bytes) {
            if (reply.reply_type == 1) {  // RI_RT_SUCC
                ROS_DEBUG("Command acknowledged successfully (seq: %u)", reply.seq_nr);
            } else {
                ROS_WARN("Command failed on robot (seq: %u, reply_type: %u)",
                         reply.seq_nr, reply.reply_type);
            }
        } else {
            ROS_WARN("Incomplete acknowledgment received: %zu of %zu bytes", received_total, expected_bytes);
        }
    }

    void heartbeatCallback(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lk(socket_mutex_);
        if (!connected_) {
            return; // No connection, skip heartbeat
        }
        
        ROS_DEBUG("Sending heartbeat to robot");
        
        // Prepare heartbeat packet
        WeldCommandPacket packet;
        memset(&packet, 0, sizeof(packet));
        
        // Header
        packet.length = sizeof(packet);
        packet.msg_type = RI_MT_WELDCMD;
        packet.comm_type = RI_CT_SVCREQ;
        packet.reply_type = RI_RT_INVAL;
        
        // Special heartbeat sequence number
        packet.seq_nr = static_cast<uint32_t>(RI_SEQ_HB);
        
        // All payload fields set to NO_CHANGE (sentinel value)
        packet.target_wire_spd = NO_CHANGE;
        packet.correction_val = NO_CHANGE;
        packet.dyn_setting = NO_CHANGE;
        packet.operation_mode = NO_CHANGE;
        packet.std_pulse_val = NO_CHANGE;
        packet.program_number = NO_CHANGE;
        packet.arc_start_cmd = NO_CHANGE;
        packet.gas_control = NO_CHANGE;
        packet.jog_feed_cmd = NO_CHANGE;
        packet.jog_retract_cmd = NO_CHANGE;
        
        // Build send buffer (convert to big-endian if required)
        uint8_t send_buf[sizeof(packet)];
        std::memcpy(send_buf, &packet, sizeof(packet));
        if (!little_endian_)
        {
            uint32_t* p = reinterpret_cast<uint32_t*>(send_buf);
            size_t cnt = sizeof(packet) / sizeof(uint32_t);
            for (size_t i = 0; i < cnt; ++i) { p[i] = htonl(p[i]); }
        }

        // Send heartbeat packet
        ssize_t bytes_sent = send(sock_fd_, send_buf, sizeof(packet), 0);
        if (bytes_sent != sizeof(packet)) {
            ROS_WARN("Failed to send heartbeat packet, disconnecting");
            disconnect();
            return;
        }

        // Try to read ACK for heartbeat (non-blocking)
        struct {
            uint32_t length;
            uint32_t msg_type;
            uint32_t comm_type;
            uint32_t reply_type;
            uint32_t seq_nr;
        } hb_reply;
        size_t expected_bytes = sizeof(hb_reply);
        size_t received_total = 0;
        while (received_total < expected_bytes) {
            ssize_t r = recv(sock_fd_, ((char*)&hb_reply)+received_total,
                             expected_bytes-received_total, MSG_DONTWAIT);
            if (r <= 0) break; // nothing to read now
            received_total += static_cast<size_t>(r);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fanuc_weld_command_node");
    
    FanucWeldCommandNode node;
    
    ROS_INFO("Fanuc Weld Command Node is running...");
    ros::spin();
    
    return 0;
} 