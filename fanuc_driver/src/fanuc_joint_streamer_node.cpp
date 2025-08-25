/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-2015, TU Delft Robotics Institute
 * Copyright (c) 2025, ROS2 Port
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "fanuc_driver/msg/joint_trajectory_point.hpp"
#include "fanuc_driver/simple_message.hpp"
#include "fanuc_driver/fanuc_utils.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>
#include <memory>

using namespace fanuc_driver::simple_message;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class FanucJointStreamerNode : public rclcpp::Node
{
public:
  FanucJointStreamerNode()
    : Node("fanuc_joint_streamer_node"),
      socket_fd_(-1),
      connected_(false),
      j23_factor_(0),
      use_bswap_(false),
      stop_thread_(false),
      sequence_counter_(0),
      streaming_active_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("robot_ip", "192.168.1.100");
    this->declare_parameter<int>("robot_port", 11000);  // Motion port
    this->declare_parameter<int>("J23_factor", 0);
    this->declare_parameter<bool>("use_bswap", false);
    this->declare_parameter<std::vector<std::string>>("joint_names",
      std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    this->declare_parameter<double>("streaming_rate", 125.0);  // Hz
    
    // Get parameters
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    robot_port_ = this->get_parameter("robot_port").as_int();
    j23_factor_ = this->get_parameter("J23_factor").as_int();
    use_bswap_ = this->get_parameter("use_bswap").as_bool();
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    streaming_rate_ = this->get_parameter("streaming_rate").as_double();
    
    // Create action server for trajectory execution
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "follow_joint_trajectory",
      std::bind(&FanucJointStreamerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FanucJointStreamerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&FanucJointStreamerNode::handleAccepted, this, std::placeholders::_1));
    
    // Subscribe to joint trajectory topic for direct streaming
    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_path_command", 10,
      std::bind(&FanucJointStreamerNode::trajectoryCallback, this, std::placeholders::_1));
    
    // Subscribe to joint states for feedback
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&FanucJointStreamerNode::jointStateCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(),
      "Fanuc Joint Streamer Node started - IP: %s, Port: %d, J23_factor: %d, ByteSwap: %s",
      robot_ip_.c_str(), robot_port_, j23_factor_, use_bswap_ ? "true" : "false");
    
    // Start streaming thread
    streaming_thread_ = std::thread(&FanucJointStreamerNode::streamingThread, this);
  }
  
  ~FanucJointStreamerNode()
  {
    stop_thread_ = true;
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
    if (streaming_thread_.joinable()) {
      streaming_thread_.join();
    }
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received trajectory goal with %zu points",
                goal->trajectory.points.size());
    
    // Validate trajectory
    if (goal->trajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting empty trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Trajectory execution cancel requested");
    
    // Send stop command to robot
    sendStopCommand();
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void handleAccepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    // Execute trajectory in a separate thread
    std::thread{std::bind(&FanucJointStreamerNode::executeTrajectory, this, goal_handle)}.detach();
  }
  
  void executeTrajectory(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    
    // Send start trajectory command
    sendStartTrajectoryCommand();
    
    // Stream trajectory points
    auto start_time = this->now();
    for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
      // Check if goal is canceling
      if (goal_handle->is_canceling()) {
        sendStopCommand();
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");
        return;
      }
      
      const auto& point = goal->trajectory.points[i];
      
      // Apply J2-J3 inverse transform (negative factor for sending to robot)
      trajectory_msgs::msg::JointTrajectoryPoint transformed_point;
      fanuc::utils::linkage_transform(point, &transformed_point, -j23_factor_);
      
      // Send point to robot
      if (!sendTrajectoryPoint(transformed_point, i)) {
        result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
        result->error_string = "Failed to send trajectory point";
        goal_handle->abort(result);
        return;
      }
      
      // Update feedback
      feedback->header.stamp = this->now();
      feedback->joint_names = joint_names_;
      feedback->desired = point;
      feedback->actual = current_joint_state_;
      goal_handle->publish_feedback(feedback);
      
      // Wait for the appropriate time (simplified timing)
      if (i < goal->trajectory.points.size() - 1) {
        auto next_time = rclcpp::Duration(goal->trajectory.points[i+1].time_from_start);
        auto current_time = this->now() - start_time;
        auto sleep_duration = next_time - current_time;
        if (sleep_duration.nanoseconds() > 0) {
          std::this_thread::sleep_for(
            std::chrono::nanoseconds(sleep_duration.nanoseconds()));
        }
      }
    }
    
    // Send end trajectory command
    sendEndTrajectoryCommand();
    
    // Set success result
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
  }
  
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", msg->points.size());
    
    // Queue trajectory points for streaming
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    // Clear existing queue and add new points
    while (!trajectory_queue_.empty()) {
      trajectory_queue_.pop();
    }
    
    for (const auto& point : msg->points) {
      trajectory_queue_.push(point);
    }
    
    streaming_active_ = true;
  }
  
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Store current joint state for feedback
    current_joint_state_.positions = msg->position;
    current_joint_state_.velocities = msg->velocity;
    current_joint_state_.accelerations = msg->effort;  // Using effort as placeholder
    current_joint_state_.time_from_start = rclcpp::Duration(0, 0);
  }
  
  void streamingThread()
  {
    auto rate = rclcpp::Rate(streaming_rate_);
    
    while (!stop_thread_ && rclcpp::ok()) {
      if (!connected_) {
        if (!connectToRobot()) {
          std::this_thread::sleep_for(std::chrono::seconds(5));
          continue;
        }
      }
      
      // Stream points if available
      if (streaming_active_) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!trajectory_queue_.empty()) {
          auto point = trajectory_queue_.front();
          trajectory_queue_.pop();
          
          // Apply J2-J3 inverse transform
          trajectory_msgs::msg::JointTrajectoryPoint transformed_point;
          fanuc::utils::linkage_transform(point, &transformed_point, -j23_factor_);
          
          sendTrajectoryPoint(transformed_point, sequence_counter_++);
          
          if (trajectory_queue_.empty()) {
            streaming_active_ = false;
          }
        }
      }
      
      rate.sleep();
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
    
    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s:%d",
                   robot_ip_.c_str(), robot_port_);
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to robot motion port at %s:%d",
                robot_ip_.c_str(), robot_port_);
    return true;
  }
  
  bool sendTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& point, int sequence)
  {
    if (!connected_) {
      return false;
    }
    
    JointTrajectoryPointMessage msg;
    msg.sequence = sequence;
    
    // Convert to simple message format
    std::vector<double> velocities = point.velocities.empty() ? 
      std::vector<double>(point.positions.size(), 0.0) : point.velocities;
    
    double duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    msg.setJointData(point.positions, velocities, duration);
    
    // Serialize and send
    auto buffer = msg.serialize(use_bswap_);
    ssize_t sent = send(socket_fd_, buffer.data(), buffer.size(), 0);
    
    if (sent != static_cast<ssize_t>(buffer.size())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send trajectory point");
      connected_ = false;
      return false;
    }
    
    return true;
  }
  
  void sendStartTrajectoryCommand()
  {
    JointTrajectoryPointMessage msg;
    msg.sequence = START_TRAJECTORY_STREAMING;
    msg.setJointData(std::vector<double>(6, 0.0), std::vector<double>(6, 0.0), 0.0);
    
    auto buffer = msg.serialize(use_bswap_);
    send(socket_fd_, buffer.data(), buffer.size(), 0);
  }
  
  void sendEndTrajectoryCommand()
  {
    JointTrajectoryPointMessage msg;
    msg.sequence = END_TRAJECTORY;
    msg.setJointData(std::vector<double>(6, 0.0), std::vector<double>(6, 0.0), 0.0);
    
    auto buffer = msg.serialize(use_bswap_);
    send(socket_fd_, buffer.data(), buffer.size(), 0);
  }
  
  void sendStopCommand()
  {
    JointTrajectoryPointMessage msg;
    msg.sequence = STOP_TRAJECTORY;
    msg.setJointData(std::vector<double>(6, 0.0), std::vector<double>(6, 0.0), 0.0);
    
    auto buffer = msg.serialize(use_bswap_);
    send(socket_fd_, buffer.data(), buffer.size(), 0);
  }
  
  // Member variables
  std::string robot_ip_;
  int robot_port_;
  int j23_factor_;
  bool use_bswap_;
  std::vector<std::string> joint_names_;
  double streaming_rate_;
  
  int socket_fd_;
  std::atomic<bool> connected_;
  std::atomic<bool> stop_thread_;
  std::atomic<bool> streaming_active_;
  std::atomic<int> sequence_counter_;
  
  std::thread streaming_thread_;
  std::mutex queue_mutex_;
  std::queue<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_queue_;
  trajectory_msgs::msg::JointTrajectoryPoint current_joint_state_;
  
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FanucJointStreamerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}