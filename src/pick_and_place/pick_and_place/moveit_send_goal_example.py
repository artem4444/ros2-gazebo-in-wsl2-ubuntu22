#!/usr/bin/env python3
"""
Example: send a joint-space goal to MoveIt2 move_group.

Usage (with sim and move_group running):
  ros2 run pick_and_place moveit_send_goal_example

Or with a custom goal from command line (joint positions in radians, space-separated):
  ros2 run pick_and_place moveit_send_goal_example -- 0.0 0.5 -0.8 0.3 0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, WorkspaceParameters
from std_msgs.msg import Header


# Default: same joint order as arm_5dof_moveit_config / ros2_controllers
ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
]


def make_joint_goal(joint_names, positions, frame_id='base_link'):
    """Build a MoveGroup.Goal for a joint-space target."""
    header = Header()
    header.frame_id = frame_id

    workspace = WorkspaceParameters()
    workspace.header = header

    constraints = Constraints()
    constraints.name = 'joint_goal'
    for name, pos in zip(joint_names, positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(pos)
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

    request = MotionPlanRequest()
    request.workspace_parameters = workspace
    request.group_name = 'arm'
    request.num_planning_attempts = 10
    request.allowed_planning_time = 5.0
    request.goal_constraints = [constraints]

    goal = MoveGroup.Goal()
    goal.request = request
    return goal


class MoveItSendGoalExample(Node):
    def __init__(self):
        super().__init__('moveit_send_goal_example')
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action...')
        self._client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Connected to MoveIt2 move_group.')

    def send_goal(self, joint_positions):
        goal_msg = make_joint_goal(ARM_JOINT_NAMES, joint_positions)
        self.get_logger().info('Sending joint goal: %s' % [round(x, 3) for x in joint_positions])
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.result().accepted:
            self.get_logger().error('Goal rejected.')
            return False
        handle = future.result()
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result().result
        err = getattr(result.error_code, 'val', result.error_code) if hasattr(result, 'error_code') else '?'
        self.get_logger().info('MoveIt result: error_code=%s (1=SUCCESS)' % err)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MoveItSendGoalExample()

    # Parse optional joint positions from argv (radians)
    import sys
    argv = sys.argv[1:]
    if argv and argv[0] == '--':
        argv = argv[1:]
    if len(argv) >= 5:
        positions = [float(x) for x in argv[:5]]
    else:
        # Default: "home" pose
        positions = [0.0, 0.0, 0.0, 0.0, 0.0]

    node.send_goal(positions)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
