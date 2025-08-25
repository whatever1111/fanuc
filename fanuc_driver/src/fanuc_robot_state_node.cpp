/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-2015, TU Delft Robotics Institute
 * Copyright (c) 2025, ROS2 Port
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
 *  * Neither the name of the TU Delft Robotics Institute nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include "fanuc_driver/msg/robot_status.hpp"
#include "fanuc_driver/msg/joint_position.hpp"
#include "fanuc_driver/simple_message.hpp"
#include "fanuc_driver/fanuc_utils.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <string>

using namespace fanuc_driver::simple_message;

class FanucRobotStateNode : public rclcpp::Node
{
public:
  FanucRobotStateNode()
    : Node("fanuc_robot_state_node"),
      socket_fd_(-1),
      connected_(false),
      j23_factor_(0),
      use_bswap_(false),
      stop_thread_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("robot_ip", "192.168.1.100");
    this->declare_parameter<int>("robot_port", 11002);  // State port
    this->declare_parameter<int>("J23_factor", 0);
    this->declare_parameter<bool>("use_bswap", false);
    this->declare_parameter<std::vector<std::string>>("joint_names", 
      std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    
    // Get parameters
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    robot_port_ = this->get_parameter("robot_port").as_int();
    j23_factor_ = this->get_parameter("J23_factor").as_int();
    use_bswap_ = this->get_parameter("use_bswap").as_bool();
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    
    // Create publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    robot_status_pub_ = this->create_publisher<fanuc_driver::msg::RobotStatus>(
      "robot_status", 10);
    feedback_pub_ = this->create_publisher<fanuc_driver::msg::JointPosition>(
      "joint_feedback", 10);
    
    RCLCPP_INFO(this->get_logger(), 
      "Fanuc Robot State Node started - IP: %s, Port: %d, J23_factor: %d, ByteSwap: %s",
      robot_ip_.c_str(), robot_port_, j23_factor_, use_bswap_ ? "true" : "false");
    
    // Start receiver thread
    receiver_thread_ = std::thread(&FanucRobotStateNode::receiverThread, this);
  }
  
  ~FanucRobotStateNode()
  {
    stop_thread_ = true;
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }

private:
  void receiverThread()
  {
    while (!stop_thread_ && rclcpp::ok()) {
      if (!connected_) {
        if (!connectToRobot()) {
          std::this_thread::sleep_for(std::chrono::seconds(5));
          continue;
        }
      }
      
      // Try to receive message
      if (!receiveAndProcessMessage()) {
        RCLCPP_WARN(this->get_logger(), "Failed to receive message, reconnecting...");
        disconnect();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }
  
  bool connectToRobot()
  {
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
      return false;
    }
    
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(robot_port_);
    
    if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid IP address: %s", robot_ip_.c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    // Set timeout for socket operations
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s:%d", 
                   robot_ip_.c_str(), robot_port_);
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to robot at %s:%d", 
                robot_ip_.c_str(), robot_port_);
    return true;
  }
  
  void disconnect()
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    connected_ = false;
  }
  
  bool receiveAndProcessMessage()
  {
    // Read prefix (4 bytes for packet length)
    uint8_t prefix_buffer[PREFIX_SIZE];
    ssize_t bytes_read = recv(socket_fd_, prefix_buffer, PREFIX_SIZE, MSG_WAITALL);
    if (bytes_read != PREFIX_SIZE) {
      return false;
    }
    
    // Parse prefix
    Prefix prefix;
    prefix.deserialize(prefix_buffer, use_bswap_);
    
    // Validate packet length
    if (prefix.packet_length < 0 || prefix.packet_length > 1024) {
      RCLCPP_WARN(this->get_logger(), "Invalid packet length: %d", prefix.packet_length);
      return false;
    }
    
    // Read the rest of the message
    std::vector<uint8_t> message_buffer(prefix.packet_length);
    bytes_read = recv(socket_fd_, message_buffer.data(), prefix.packet_length, MSG_WAITALL);
    if (bytes_read != prefix.packet_length) {
      return false;
    }
    
    // Parse header to determine message type
    if (message_buffer.size() < HEADER_SIZE) {
      return false;
    }
    
    Header header;
    header.deserialize(message_buffer.data(), use_bswap_);
    
    // Process based on message type
    switch (static_cast<MsgType>(header.msg_type)) {
      case MsgType::JOINT_POSITION:
      case MsgType::JOINT_FEEDBACK:
        processJointMessage(message_buffer.data(), message_buffer.size());
        break;
      case MsgType::STATUS:
        processStatusMessage(message_buffer.data(), message_buffer.size());
        break;
      default:
        RCLCPP_DEBUG(this->get_logger(), "Received message type: %d", header.msg_type);
        break;
    }
    
    return true;
  }
  
  void processJointMessage(const uint8_t* buffer, size_t size)
  {
    JointPositionMessage msg;
    if (!msg.deserialize(buffer, size, use_bswap_)) {
      RCLCPP_WARN(this->get_logger(), "Failed to deserialize joint message");
      return;
    }
    
    // Get joint positions
    std::vector<double> positions = msg.getJointPositions();
    
    RCLCPP_DEBUG(this->get_logger(), "Received %zu joint positions from SimpleMessage", positions.size());
    
    // Apply J2-J3 linkage transform
    std::vector<double> transformed_positions;
    fanuc::utils::linkage_transform(positions, &transformed_positions, j23_factor_);
    
    // SimpleMessage sends MAX_JOINTS (10) positions, but we may only use some of them
    // We should have at least as many positions as configured joint names
    if (transformed_positions.size() < joint_names_.size()) {
      RCLCPP_ERROR(this->get_logger(), 
        "Received %zu joint positions but expected at least %zu. Message will be dropped.",
        transformed_positions.size(), joint_names_.size());
      return;
    }
    
    // Publish sensor_msgs/JointState
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = joint_names_;
    
    // Use exactly the number of joints defined in joint_names
    joint_state_msg->position.assign(
      transformed_positions.begin(), 
      transformed_positions.begin() + joint_names_.size()
    );
    
    joint_state_pub_->publish(std::move(joint_state_msg));
    
    // Also publish custom JointPosition message for compatibility
    auto feedback_msg = std::make_unique<fanuc_driver::msg::JointPosition>();
    feedback_msg->header.stamp = this->now();
    feedback_msg->sequence = msg.sequence;
    feedback_msg->positions = transformed_positions;
    
    feedback_pub_->publish(std::move(feedback_msg));
  }
  
  void processStatusMessage(const uint8_t* buffer, size_t size)
  {
    RobotStatusMessage msg;
    if (!msg.deserialize(buffer, size, use_bswap_)) {
      RCLCPP_WARN(this->get_logger(), "Failed to deserialize status message");
      return;
    }
    
    // Publish robot status
    auto status_msg = std::make_unique<fanuc_driver::msg::RobotStatus>();
    status_msg->header.stamp = this->now();
    status_msg->mode = msg.mode;
    status_msg->e_stopped = msg.e_stopped;
    status_msg->drives_powered = msg.drives_powered;
    status_msg->motion_possible = msg.motion_possible;
    status_msg->in_motion = msg.in_motion;
    status_msg->in_error = msg.in_error;
    status_msg->error_code = msg.error_code;
    
    robot_status_pub_->publish(std::move(status_msg));
    
    // Log status changes
    static int8_t last_mode = -1;
    static int8_t last_motion = -1;
    if (msg.mode != last_mode || msg.in_motion != last_motion) {
      RCLCPP_INFO(this->get_logger(), 
        "Robot Status - Mode: %d, Motion: %d, Powered: %d, E-Stop: %d",
        msg.mode, msg.in_motion, msg.drives_powered, msg.e_stopped);
      last_mode = msg.mode;
      last_motion = msg.in_motion;
    }
  }
  
  // Member variables
  std::string robot_ip_;
  int robot_port_;
  int j23_factor_;
  bool use_bswap_;
  std::vector<std::string> joint_names_;
  
  int socket_fd_;
  std::atomic<bool> connected_;
  std::atomic<bool> stop_thread_;
  std::thread receiver_thread_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<fanuc_driver::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<fanuc_driver::msg::JointPosition>::SharedPtr feedback_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FanucRobotStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}