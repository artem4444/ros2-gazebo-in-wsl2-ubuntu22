// Copyright 2025 OpenArm Bimanual Control
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENARM_BIMANUAL_CONTROL__TRAJECTORY_MERGER_HPP_
#define OPENARM_BIMANUAL_CONTROL__TRAJECTORY_MERGER_HPP_

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>
#include <string>

namespace openarm_bimanual_control
{

/**
 * Linear interpolation of a JointTrajectory at time t.
 * Fills out_positions with interpolated joint positions.
 * \return true if interpolation succeeded, false if traj empty or t out of range.
 */
bool interpolateAtTime(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double t,
  std::vector<double> & out_positions);

/**
 * Merge multiple joint trajectories onto a common time axis.
 * Resamples all trajectories at times 0, dt, 2*dt, ... up to max duration.
 * \param trajectories Per-group joint trajectories (e.g. left_arm, right_arm).
 * \param joint_names_per_group Joint names for each trajectory (same order as trajectories).
 * \param dt Time step for resampling (seconds).
 * \param merged Output: one trajectory with concatenated joint names and shared time_from_start.
 */
void mergeTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & trajectories,
  const std::vector<std::vector<std::string>> & joint_names_per_group,
  double dt,
  trajectory_msgs::msg::JointTrajectory & merged);

/**
 * Extract a sub-trajectory for a subset of joints (same time axis, only those positions).
 * Used to split a merged trajectory for separate controllers (e.g. left_arm only).
 */
void extractSubTrajectory(
  const trajectory_msgs::msg::JointTrajectory & merged,
  const std::vector<std::string> & joint_names,
  trajectory_msgs::msg::JointTrajectory & out);

}  // namespace openarm_bimanual_control

#endif  // OPENARM_BIMANUAL_CONTROL__TRAJECTORY_MERGER_HPP_
