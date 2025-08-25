/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * Copyright (c) 2025, ROS2 Port
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 */

#ifndef FANUC_DRIVER__FANUC_UTILS_HPP_
#define FANUC_DRIVER__FANUC_UTILS_HPP_

#include <vector>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace fanuc
{
namespace utils
{

/**
 * \brief Corrects for parallel linkage coupling between joints.
 *
 * \param[in] joints_in input joint angles
 * \param[out] joints_out output joint angles
 * \param[in] J23_factor  Linkage factor for J2-J3.
 *   J3_out = J3_in + j23_factor * J2_in
 */
void linkage_transform(const std::vector<double>& joints_in,
    std::vector<double>* joints_out, double J23_factor = 0);

/**
 * \brief Corrects for parallel linkage coupling between joints.
 *
 * \param[in] pt_in input joint trajectory point
 * \param[out] pt_out output joint trajectory point
 * \param[in] J23_factor  Linkage factor for J2-J3.
 *   J3_out = J3_in + j23_factor * J2_in
 */
void linkage_transform(const trajectory_msgs::msg::JointTrajectoryPoint& pt_in,
    trajectory_msgs::msg::JointTrajectoryPoint* pt_out, double J23_factor = 0);

} // namespace utils
} // namespace fanuc

#endif // FANUC_DRIVER__FANUC_UTILS_HPP_