/*
 * Software License Agreement (BSD License)
 *
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
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 */

#ifndef FANUC_DRIVER__SIMPLE_MESSAGE_HPP_
#define FANUC_DRIVER__SIMPLE_MESSAGE_HPP_

#include <cstdint>
#include <vector>
#include <cstring>
#include <arpa/inet.h>

namespace fanuc_driver
{
namespace simple_message
{

// Standard message types
enum class MsgType : int32_t
{
  INVALID = 0,
  PING = 1,
  
  // Robot messages
  JOINT_POSITION = 10,
  JOINT_TRAJ_PT = 11,
  JOINT_TRAJ = 12,
  STATUS = 13,
  JOINT_TRAJ_PT_FULL = 14,
  JOINT_FEEDBACK = 15,
  
  // I/O messages  
  READ_INPUT = 20,
  READ_OUTPUT = 21,
  WRITE_OUTPUT = 22,
  
  // Vendor specific
  SWRI_MSG_BEGIN = 1000,
  UR_MSG_BEGIN = 1100,
  ADEPT_MSG_BEGIN = 1200,
  ABB_MSG_BEGIN = 1300,
  FANUC_MSG_BEGIN = 1400,
  MOTOMAN_MSG_BEGIN = 2000
};

// Communication types
enum class CommType : int32_t
{
  INVALID = 0,
  TOPIC = 1,
  SERVICE_REQUEST = 2,
  SERVICE_REPLY = 3
};

// Reply codes
enum class ReplyCode : int32_t
{
  INVALID = 0,
  SUCCESS = 1,
  FAILURE = 2
};

// Special sequence values
constexpr int32_t START_TRAJECTORY_DOWNLOAD = -1;
constexpr int32_t START_TRAJECTORY_STREAMING = -2;
constexpr int32_t END_TRAJECTORY = -3;
constexpr int32_t STOP_TRAJECTORY = -4;

// Maximum sizes
constexpr size_t MAX_JOINTS = 10;
constexpr size_t PREFIX_SIZE = 4;  // sizeof(int32_t) for packet length
constexpr size_t HEADER_SIZE = 12; // 3 * sizeof(int32_t) 

struct Header
{
  int32_t msg_type;
  int32_t comm_type;
  int32_t reply_code;
  
  void serialize(uint8_t* buffer, bool swap_bytes = false) const;
  void deserialize(const uint8_t* buffer, bool swap_bytes = false);
};

struct Prefix
{
  int32_t packet_length;
  
  void serialize(uint8_t* buffer, bool swap_bytes = false) const;
  void deserialize(const uint8_t* buffer, bool swap_bytes = false);
};

// Base message class
class Message
{
public:
  Header header;
  std::vector<uint8_t> data;
  
  Message() = default;
  virtual ~Message() = default;
  
  // Serialize complete message with prefix
  std::vector<uint8_t> serialize(bool swap_bytes = false) const;
  
  // Deserialize from buffer (without prefix)
  bool deserialize(const uint8_t* buffer, size_t size, bool swap_bytes = false);
  
  // Get total size including prefix and header
  size_t getTotalSize() const { return PREFIX_SIZE + HEADER_SIZE + data.size(); }
  
protected:
  virtual void serializeData(bool swap_bytes = false) {}
  virtual void deserializeData(bool swap_bytes = false) {}
};

// Joint position message
class JointPositionMessage : public Message
{
public:
  int32_t sequence;
  std::vector<float> joint_data;  // Max 10 joints
  
  JointPositionMessage();
  
  void setJointPositions(const std::vector<double>& positions);
  std::vector<double> getJointPositions() const;
  
protected:
  void serializeData(bool swap_bytes = false) override;
  void deserializeData(bool swap_bytes = false) override;
};

// Joint trajectory point message
class JointTrajectoryPointMessage : public Message
{
public:
  int32_t sequence;
  std::vector<float> joint_data;     // positions
  std::vector<float> velocity_data;  // velocities
  float duration;
  
  JointTrajectoryPointMessage();
  
  void setJointData(const std::vector<double>& positions,
                     const std::vector<double>& velocities,
                     double duration_sec);
                     
protected:
  void serializeData(bool swap_bytes = false) override;
  void deserializeData(bool swap_bytes = false) override;
};

// Robot status message
class RobotStatusMessage : public Message
{
public:
  int8_t drives_powered;
  int8_t e_stopped;
  int32_t error_code;
  int8_t in_error;
  int8_t in_motion;
  int8_t mode;
  int8_t motion_possible;
  
  RobotStatusMessage();
  
protected:
  void serializeData(bool swap_bytes = false) override;
  void deserializeData(bool swap_bytes = false) override;
};

// Utility functions
inline int32_t swap32(int32_t value)
{
  return static_cast<int32_t>(htonl(static_cast<uint32_t>(value)));
}

inline float swapFloat(float value)
{
  uint32_t temp;
  std::memcpy(&temp, &value, sizeof(temp));
  temp = htonl(temp);
  float result;
  std::memcpy(&result, &temp, sizeof(result));
  return result;
}

} // namespace simple_message
} // namespace fanuc_driver

#endif // FANUC_DRIVER__SIMPLE_MESSAGE_HPP_