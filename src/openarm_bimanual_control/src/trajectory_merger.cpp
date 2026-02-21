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

#include "openarm_bimanual_control/trajectory_merger.hpp"
#include <builtin_interfaces/msg/duration.hpp>
#include <algorithm>
#include <cmath>
#include <map>

namespace openarm_bimanual_control
{

static double durationToSeconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

static builtin_interfaces::msg::Duration secondsToDuration(double t)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(std::floor(t));
  d.nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));
  return d;
}

bool interpolateAtTime(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double t,
  std::vector<double> & out_positions)
{
  if (traj.points.empty() || traj.joint_names.empty()) {
    return false;
  }

  const size_t n_joints = traj.joint_names.size();
  out_positions.resize(n_joints);

  const double t0 = durationToSeconds(traj.points.front().time_from_start);
  const double t1 = durationToSeconds(traj.points.back().time_from_start);

  if (t <= t0) {
    for (size_t i = 0; i < n_joints; ++i) {
      out_positions[i] = traj.points.front().positions[i];
    }
    return true;
  }
  if (t >= t1) {
    for (size_t i = 0; i < n_joints; ++i) {
      out_positions[i] = traj.points.back().positions[i];
    }
    return true;
  }

  size_t idx = 0;
  for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
    double ti = durationToSeconds(traj.points[i].time_from_start);
    double ti1 = durationToSeconds(traj.points[i + 1].time_from_start);
    if (t >= ti && t <= ti1) {
      idx = i;
      break;
    }
  }

  double ta = durationToSeconds(traj.points[idx].time_from_start);
  double tb = durationToSeconds(traj.points[idx + 1].time_from_start);
  double frac = (std::abs(tb - ta) > 1e-9) ? (t - ta) / (tb - ta) : 1.0;

  for (size_t j = 0; j < n_joints; ++j) {
    double pa = traj.points[idx].positions[j];
    double pb = traj.points[idx + 1].positions[j];
    out_positions[j] = pa + frac * (pb - pa);
  }
  return true;
}

void mergeTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & trajectories,
  const std::vector<std::vector<std::string>> & joint_names_per_group,
  double dt,
  trajectory_msgs::msg::JointTrajectory & merged)
{
  merged.points.clear();
  merged.joint_names.clear();

  if (trajectories.empty() || trajectories.size() != joint_names_per_group.size()) {
    return;
  }

  for (const auto & names : joint_names_per_group) {
    merged.joint_names.insert(merged.joint_names.end(), names.begin(), names.end());
  }

  double t_max = 0.0;
  for (const auto & traj : trajectories) {
    if (!traj.points.empty()) {
      double t_end = durationToSeconds(traj.points.back().time_from_start);
      if (t_end > t_max) {
        t_max = t_end;
      }
    }
  }

  if (dt <= 0.0) {
    dt = 0.02;
  }
  const size_t num_steps = static_cast<size_t>(std::ceil(t_max / dt)) + 1;

  std::vector<double> positions;
  for (size_t k = 0; k < num_steps; ++k) {
    const double t = k * dt;
    positions.clear();

    for (size_t g = 0; g < trajectories.size(); ++g) {
      std::vector<double> group_pos;
      if (!interpolateAtTime(trajectories[g], t, group_pos)) {
        group_pos.resize(joint_names_per_group[g].size(), 0.0);
      }
      positions.insert(positions.end(), group_pos.begin(), group_pos.end());
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start = secondsToDuration(t);
    pt.positions = positions;
    merged.points.push_back(pt);
  }
}

void extractSubTrajectory(
  const trajectory_msgs::msg::JointTrajectory & merged,
  const std::vector<std::string> & joint_names,
  trajectory_msgs::msg::JointTrajectory & out)
{
  out.joint_names = joint_names;
  out.points.clear();

  std::map<std::string, size_t> merged_index;
  for (size_t i = 0; i < merged.joint_names.size(); ++i) {
    merged_index[merged.joint_names[i]] = i;
  }

  for (const auto & pt : merged.points) {
    trajectory_msgs::msg::JointTrajectoryPoint out_pt;
    out_pt.time_from_start = pt.time_from_start;
    for (const auto & jname : joint_names) {
      auto it = merged_index.find(jname);
      if (it != merged_index.end() && it->second < pt.positions.size()) {
        out_pt.positions.push_back(pt.positions[it->second]);
      }
    }
    if (out_pt.positions.size() == joint_names.size()) {
      out.points.push_back(out_pt);
    }
  }
}

}  // namespace openarm_bimanual_control
