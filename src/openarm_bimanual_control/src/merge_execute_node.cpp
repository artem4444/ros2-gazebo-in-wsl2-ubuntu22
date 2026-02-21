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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "openarm_bimanual_control/trajectory_merger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

namespace openarm_bimanual_control
{

class MergeExecuteNode : public rclcpp::Node
{
public:
  explicit MergeExecuteNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("merge_execute_node", options)
  {
    declare_parameter("left_planning_group", "left_arm");
    declare_parameter("right_planning_group", "right_arm");
    declare_parameter("left_controller_action", "/left_joint_trajectory_controller/follow_joint_trajectory");
    declare_parameter("right_controller_action", "/right_joint_trajectory_controller/follow_joint_trajectory");
    declare_parameter("merge_dt", 0.02);
    declare_parameter("plan_to_named_state", "hands_up");

    left_planning_group_ = get_parameter("left_planning_group").as_string();
    right_planning_group_ = get_parameter("right_planning_group").as_string();
    left_controller_action_ = get_parameter("left_controller_action").as_string();
    right_controller_action_ = get_parameter("right_controller_action").as_string();
    merge_dt_ = get_parameter("merge_dt").as_double();
    plan_to_named_state_ = get_parameter("plan_to_named_state").as_string();

    left_joint_names_ = {
      "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
      "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
      "openarm_left_joint7"
    };
    right_joint_names_ = {
      "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
      "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
      "openarm_right_joint7"
    };

    left_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, left_controller_action_);
    right_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, right_controller_action_);

    timer_ = create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&MergeExecuteNode::runOnce, this));
  }

private:
  void runOnce()
  {
    timer_->cancel();

    moveit::planning_interface::MoveGroupInterface left_group(
      shared_from_this(), left_planning_group_);
    moveit::planning_interface::MoveGroupInterface right_group(
      shared_from_this(), right_planning_group_);

    left_group.setPlanningTime(5.0);
    right_group.setPlanningTime(5.0);

    left_group.setNamedTarget(plan_to_named_state_);
    right_group.setNamedTarget(plan_to_named_state_);

    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;

    RCLCPP_INFO(get_logger(), "Planning left arm...");
    auto left_result = left_group.plan(left_plan);
    if (left_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Left arm planning failed (code %d)", left_result.val);
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning right arm...");
    auto right_result = right_group.plan(right_plan);
    if (right_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Right arm planning failed (code %d)", right_result.val);
      return;
    }

    if (left_plan.trajectory_.joint_trajectory.points.empty() ||
      right_plan.trajectory_.joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(get_logger(), "Empty trajectory from planner");
      return;
    }

    std::vector<trajectory_msgs::msg::JointTrajectory> trajectories = {
      left_plan.trajectory_.joint_trajectory,
      right_plan.trajectory_.joint_trajectory
    };
    std::vector<std::vector<std::string>> joint_names_per_group = {
      left_joint_names_,
      right_joint_names_
    };

    trajectory_msgs::msg::JointTrajectory merged;
    mergeTrajectories(trajectories, joint_names_per_group, merge_dt_, merged);

    trajectory_msgs::msg::JointTrajectory left_traj;
    trajectory_msgs::msg::JointTrajectory right_traj;
    extractSubTrajectory(merged, left_joint_names_, left_traj);
    extractSubTrajectory(merged, right_joint_names_, right_traj);

    RCLCPP_INFO(get_logger(), "Sending synchronized trajectories to controllers...");

    auto left_goal = std::make_shared<FollowJointTrajectory::Goal>();
    left_goal->trajectory = left_traj;
    auto right_goal = std::make_shared<FollowJointTrajectory::Goal>();
    right_goal->trajectory = right_traj;

    if (!left_action_client_->wait_for_action_server(std::chrono::seconds(2)) ||
      !right_action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(get_logger(), "Action servers not available");
      return;
    }

    auto left_future = left_action_client_->async_send_goal(*left_goal);
    auto right_future = right_action_client_->async_send_goal(*right_goal);

    rclcpp::spin_until_future_complete(get_node_base_interface(), left_future, std::chrono::seconds(1));
    rclcpp::spin_until_future_complete(get_node_base_interface(), right_future, std::chrono::seconds(1));

    auto left_handle = left_future.get();
    auto right_handle = right_future.get();
    if (!left_handle || !right_handle) {
      RCLCPP_ERROR(get_logger(), "Failed to send goals");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goals sent. Waiting for execution...");
    auto left_result_future = left_action_client_->async_get_result(left_handle);
    auto right_result_future = right_action_client_->async_get_result(right_handle);

    rclcpp::spin_until_future_complete(get_node_base_interface(), left_result_future, std::chrono::seconds(30));
    rclcpp::spin_until_future_complete(get_node_base_interface(), right_result_future, std::chrono::seconds(30));

    auto left_result_msg = left_result_future.get();
    auto right_result_msg = right_result_future.get();
    if (left_result_msg.result && right_result_msg.result) {
      RCLCPP_INFO(get_logger(), "Bimanual execution completed.");
    } else {
      RCLCPP_WARN(get_logger(), "Execution may have failed (check action results).");
    }
  }

  std::string left_planning_group_;
  std::string right_planning_group_;
  std::string left_controller_action_;
  std::string right_controller_action_;
  double merge_dt_;
  std::string plan_to_named_state_;
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_action_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace openarm_bimanual_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openarm_bimanual_control::MergeExecuteNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
