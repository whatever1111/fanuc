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

#include <ros/ros.h>
#include <fanuc_driver/WeldCommand.h>
#include <std_msgs/Header.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

class FanucWeldCommandNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    
    std::string robot_ip_;
    int robot_port_;
    int sock_fd_;
    bool connected_;
    
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
    static const int32_t NO_CHANGE = -1;      // Sentinel value for unchanged parameters
    
    uint32_t sequence_number_;

public:
    FanucWeldCommandNode() : nh_("~"), sock_fd_(-1), connected_(false), sequence_number_(0)
    {
        // Get parameters
        nh_.param<std::string>("robot_ip", robot_ip_, "127.0.0.1");
        nh_.param<int>("robot_port", robot_port_, 11002);
        
        ROS_INFO("Fanuc Weld Command Node starting");
        ROS_INFO("Target robot: %s:%d", robot_ip_.c_str(), robot_port_);
        
        // Subscribe to weld command topic
        command_sub_ = nh_.subscribe("/weld_command", 10, 
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
        server_addr.sin_port = htons(robot_port_);
        
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
        ROS_INFO("Received weld command: wire_spd=%d, mode=%d, prog=%d", 
                 msg->target_wire_spd, msg->operation_mode, msg->program_number);
        
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
        
        // Initialize payload to sentinel values (NO_CHANGE = -1)
        // This ensures parameters not explicitly set won't overwrite robot settings
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
        
        // Copy values from ROS message (all values are copied directly)
        // To leave a parameter unchanged, set it to -1 in the ROS message
        packet.target_wire_spd = msg->target_wire_spd;
        packet.correction_val = msg->correction_val;
        packet.dyn_setting = msg->dyn_setting;
        packet.operation_mode = msg->operation_mode;
        packet.std_pulse_val = msg->std_pulse_val;
        packet.program_number = msg->program_number;
        packet.arc_start_cmd = msg->arc_start_cmd;
        packet.gas_control = msg->gas_control;
        packet.jog_feed_cmd = msg->jog_feed_cmd;
        packet.jog_retract_cmd = msg->jog_retract_cmd;
        
        // Send packet
        ssize_t bytes_sent = send(sock_fd_, &packet, sizeof(packet), 0);
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
        
        ssize_t bytes_received = recv(sock_fd_, &reply, sizeof(reply), 0);
        if (bytes_received == sizeof(reply)) {
            if (reply.reply_type == 1) {  // RI_RT_SUCC
                ROS_DEBUG("Command acknowledged successfully (seq: %u)", reply.seq_nr);
            } else {
                ROS_WARN("Command failed on robot (seq: %u, reply_type: %u)", 
                         reply.seq_nr, reply.reply_type);
            }
        } else {
            ROS_WARN("Failed to receive acknowledgment from robot");
            disconnect();
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