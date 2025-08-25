/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * Copyright (c) 2025, ROS2 Port
 * All rights reserved.
 */

#include "fanuc_driver/fanuc_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cassert>

namespace fanuc
{
namespace utils
{

void linkage_transform(const std::vector<double>& joints_in,
    std::vector<double>* joints_out, double J23_factor)
{
  assert(joints_in.size() >= 3);
  assert(joints_out != nullptr);
  
  *joints_out = joints_in;
  // Apply J2-J3 coupling: J3_out = J3_in + J23_factor * J2_in
  if (joints_out->size() >= 3) {
    joints_out->at(2) += J23_factor * joints_out->at(1);
  }
}

void linkage_transform(const trajectory_msgs::msg::JointTrajectoryPoint& pt_in,
    trajectory_msgs::msg::JointTrajectoryPoint* pt_out, double J23_factor)
{
  assert(pt_out != nullptr);
  
  *pt_out = pt_in;
  
  // Transform positions if available
  if (!pt_in.positions.empty()) {
    linkage_transform(pt_in.positions, &(pt_out->positions), J23_factor);
  }
  
  // Note: Velocities and accelerations are also affected by linkage
  // but for now we keep them as-is (matching ROS1 implementation)
  // Future enhancement: properly transform velocities/accelerations
}

} // namespace utils
} // namespace fanuc