/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2025, ROS2 Port
 * All rights reserved.
 */

#include "fanuc_driver/simple_message.hpp"
#include <algorithm>
#include <cstring>

namespace fanuc_driver
{
namespace simple_message
{

void Header::serialize(uint8_t* buffer, bool swap_bytes) const
{
  int32_t* ptr = reinterpret_cast<int32_t*>(buffer);
  ptr[0] = swap_bytes ? swap32(msg_type) : msg_type;
  ptr[1] = swap_bytes ? swap32(comm_type) : comm_type;
  ptr[2] = swap_bytes ? swap32(reply_code) : reply_code;
}

void Header::deserialize(const uint8_t* buffer, bool swap_bytes)
{
  const int32_t* ptr = reinterpret_cast<const int32_t*>(buffer);
  msg_type = swap_bytes ? swap32(ptr[0]) : ptr[0];
  comm_type = swap_bytes ? swap32(ptr[1]) : ptr[1];
  reply_code = swap_bytes ? swap32(ptr[2]) : ptr[2];
}

void Prefix::serialize(uint8_t* buffer, bool swap_bytes) const
{
  int32_t* ptr = reinterpret_cast<int32_t*>(buffer);
  *ptr = swap_bytes ? swap32(packet_length) : packet_length;
}

void Prefix::deserialize(const uint8_t* buffer, bool swap_bytes)
{
  const int32_t* ptr = reinterpret_cast<const int32_t*>(buffer);
  packet_length = swap_bytes ? swap32(*ptr) : *ptr;
}

std::vector<uint8_t> Message::serialize(bool swap_bytes) const
{
  size_t total_size = getTotalSize();
  std::vector<uint8_t> buffer(total_size);
  
  // Write prefix
  Prefix prefix;
  prefix.packet_length = static_cast<int32_t>(HEADER_SIZE + data.size());
  prefix.serialize(buffer.data(), swap_bytes);
  
  // Write header
  header.serialize(buffer.data() + PREFIX_SIZE, swap_bytes);
  
  // Write data
  if (!data.empty()) {
    std::memcpy(buffer.data() + PREFIX_SIZE + HEADER_SIZE, data.data(), data.size());
  }
  
  return buffer;
}

bool Message::deserialize(const uint8_t* buffer, size_t size, bool swap_bytes)
{
  if (size < HEADER_SIZE) {
    return false;
  }
  
  // Read header
  header.deserialize(buffer, swap_bytes);
  
  // Read data
  size_t data_size = size - HEADER_SIZE;
  if (data_size > 0) {
    data.resize(data_size);
    std::memcpy(data.data(), buffer + HEADER_SIZE, data_size);
    deserializeData(swap_bytes);
  }
  
  return true;
}

JointPositionMessage::JointPositionMessage()
{
  header.msg_type = static_cast<int32_t>(MsgType::JOINT_POSITION);
  header.comm_type = static_cast<int32_t>(CommType::TOPIC);
  header.reply_code = static_cast<int32_t>(ReplyCode::INVALID);
  sequence = 0;
  joint_data.resize(MAX_JOINTS, 0.0f);
}

void JointPositionMessage::setJointPositions(const std::vector<double>& positions)
{
  size_t num_joints = std::min(positions.size(), MAX_JOINTS);
  for (size_t i = 0; i < num_joints; ++i) {
    joint_data[i] = static_cast<float>(positions[i]);
  }
  for (size_t i = num_joints; i < MAX_JOINTS; ++i) {
    joint_data[i] = 0.0f;
  }
  serializeData(false);
}

std::vector<double> JointPositionMessage::getJointPositions() const
{
  std::vector<double> positions;
  positions.reserve(joint_data.size());
  for (const auto& val : joint_data) {
    positions.push_back(static_cast<double>(val));
  }
  return positions;
}

void JointPositionMessage::serializeData(bool swap_bytes)
{
  size_t data_size = sizeof(int32_t) + MAX_JOINTS * sizeof(float);
  data.resize(data_size);
  
  uint8_t* ptr = data.data();
  
  // Write sequence
  int32_t seq = swap_bytes ? swap32(sequence) : sequence;
  std::memcpy(ptr, &seq, sizeof(seq));
  ptr += sizeof(seq);
  
  // Write joint data
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    float val = swap_bytes ? swapFloat(joint_data[i]) : joint_data[i];
    std::memcpy(ptr, &val, sizeof(val));
    ptr += sizeof(val);
  }
}

void JointPositionMessage::deserializeData(bool swap_bytes)
{
  if (data.size() < sizeof(int32_t) + MAX_JOINTS * sizeof(float)) {
    return;
  }
  
  const uint8_t* ptr = data.data();
  
  // Read sequence
  int32_t seq;
  std::memcpy(&seq, ptr, sizeof(seq));
  sequence = swap_bytes ? swap32(seq) : seq;
  ptr += sizeof(seq);
  
  // Read joint data
  joint_data.resize(MAX_JOINTS);
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    float val;
    std::memcpy(&val, ptr, sizeof(val));
    joint_data[i] = swap_bytes ? swapFloat(val) : val;
    ptr += sizeof(val);
  }
}

JointTrajectoryPointMessage::JointTrajectoryPointMessage()
{
  header.msg_type = static_cast<int32_t>(MsgType::JOINT_TRAJ_PT);
  header.comm_type = static_cast<int32_t>(CommType::TOPIC);
  header.reply_code = static_cast<int32_t>(ReplyCode::INVALID);
  sequence = 0;
  joint_data.resize(MAX_JOINTS, 0.0f);
  velocity_data.resize(MAX_JOINTS, 0.0f);
  duration = 0.0f;
}

void JointTrajectoryPointMessage::setJointData(const std::vector<double>& positions,
                                                const std::vector<double>& velocities,
                                                double duration_sec)
{
  size_t num_pos = std::min(positions.size(), MAX_JOINTS);
  for (size_t i = 0; i < num_pos; ++i) {
    joint_data[i] = static_cast<float>(positions[i]);
  }
  for (size_t i = num_pos; i < MAX_JOINTS; ++i) {
    joint_data[i] = 0.0f;
  }
  
  size_t num_vel = std::min(velocities.size(), MAX_JOINTS);
  for (size_t i = 0; i < num_vel; ++i) {
    velocity_data[i] = static_cast<float>(velocities[i]);
  }
  for (size_t i = num_vel; i < MAX_JOINTS; ++i) {
    velocity_data[i] = 0.0f;
  }
  
  duration = static_cast<float>(duration_sec);
  serializeData(false);
}

void JointTrajectoryPointMessage::serializeData(bool swap_bytes)
{
  size_t data_size = sizeof(int32_t) + 2 * MAX_JOINTS * sizeof(float) + sizeof(float);
  data.resize(data_size);
  
  uint8_t* ptr = data.data();
  
  // Write sequence
  int32_t seq = swap_bytes ? swap32(sequence) : sequence;
  std::memcpy(ptr, &seq, sizeof(seq));
  ptr += sizeof(seq);
  
  // Write joint positions
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    float val = swap_bytes ? swapFloat(joint_data[i]) : joint_data[i];
    std::memcpy(ptr, &val, sizeof(val));
    ptr += sizeof(val);
  }
  
  // Write joint velocities
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    float val = swap_bytes ? swapFloat(velocity_data[i]) : velocity_data[i];
    std::memcpy(ptr, &val, sizeof(val));
    ptr += sizeof(val);
  }
  
  // Write duration
  float dur = swap_bytes ? swapFloat(duration) : duration;
  std::memcpy(ptr, &dur, sizeof(dur));
}

void JointTrajectoryPointMessage::deserializeData(bool swap_bytes)
{
  size_t expected_size = sizeof(int32_t) + 2 * MAX_JOINTS * sizeof(float) + sizeof(float);
  if (data.size() < expected_size) {
    return;
  }
  
  const uint8_t* ptr = data.data();
  
  // Read sequence
  int32_t seq;
  std::memcpy(&seq, ptr, sizeof(seq));
  sequence = swap_bytes ? swap32(seq) : seq;
  ptr += sizeof(seq);
  
  // Read joint positions
  joint_data.resize(MAX_JOINTS);
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    float val;
    std::memcpy(&val, ptr, sizeof(val));
    joint_data[i] = swap_bytes ? swapFloat(val) : val;
    ptr += sizeof(val);
  }
  
  // Read joint velocities
  velocity_data.resize(MAX_JOINTS);
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    float val;
    std::memcpy(&val, ptr, sizeof(val));
    velocity_data[i] = swap_bytes ? swapFloat(val) : val;
    ptr += sizeof(val);
  }
  
  // Read duration
  float dur;
  std::memcpy(&dur, ptr, sizeof(dur));
  duration = swap_bytes ? swapFloat(dur) : dur;
}

RobotStatusMessage::RobotStatusMessage()
{
  header.msg_type = static_cast<int32_t>(MsgType::STATUS);
  header.comm_type = static_cast<int32_t>(CommType::TOPIC);
  header.reply_code = static_cast<int32_t>(ReplyCode::INVALID);
  drives_powered = 0;
  e_stopped = 0;
  error_code = 0;
  in_error = 0;
  in_motion = 0;
  mode = 0;
  motion_possible = 0;
}

void RobotStatusMessage::serializeData(bool swap_bytes)
{
  // Robot status has specific field order in simple_message
  size_t data_size = 7 * sizeof(int32_t);  // All fields sent as int32
  data.resize(data_size);
  
  int32_t* ptr = reinterpret_cast<int32_t*>(data.data());
  
  ptr[0] = swap_bytes ? swap32(static_cast<int32_t>(drives_powered)) : static_cast<int32_t>(drives_powered);
  ptr[1] = swap_bytes ? swap32(static_cast<int32_t>(e_stopped)) : static_cast<int32_t>(e_stopped);
  ptr[2] = swap_bytes ? swap32(error_code) : error_code;
  ptr[3] = swap_bytes ? swap32(static_cast<int32_t>(in_error)) : static_cast<int32_t>(in_error);
  ptr[4] = swap_bytes ? swap32(static_cast<int32_t>(in_motion)) : static_cast<int32_t>(in_motion);
  ptr[5] = swap_bytes ? swap32(static_cast<int32_t>(mode)) : static_cast<int32_t>(mode);
  ptr[6] = swap_bytes ? swap32(static_cast<int32_t>(motion_possible)) : static_cast<int32_t>(motion_possible);
}

void RobotStatusMessage::deserializeData(bool swap_bytes)
{
  if (data.size() < 7 * sizeof(int32_t)) {
    return;
  }
  
  const int32_t* ptr = reinterpret_cast<const int32_t*>(data.data());
  
  drives_powered = static_cast<int8_t>(swap_bytes ? swap32(ptr[0]) : ptr[0]);
  e_stopped = static_cast<int8_t>(swap_bytes ? swap32(ptr[1]) : ptr[1]);
  error_code = swap_bytes ? swap32(ptr[2]) : ptr[2];
  in_error = static_cast<int8_t>(swap_bytes ? swap32(ptr[3]) : ptr[3]);
  in_motion = static_cast<int8_t>(swap_bytes ? swap32(ptr[4]) : ptr[4]);
  mode = static_cast<int8_t>(swap_bytes ? swap32(ptr[5]) : ptr[5]);
  motion_possible = static_cast<int8_t>(swap_bytes ? swap32(ptr[6]) : ptr[6]);
}

} // namespace simple_message
} // namespace fanuc_driver