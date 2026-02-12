#!/usr/bin/env python3
"""
Bridge node: forwards desired joint states from the GUI sliders to ros2_control.

- Subscribes to /joint_states_desired (published by joint_state_publisher_gui with remap).
- Sends arm positions to /arm_controller/joint_trajectory (topic).
- Sends gripper position to /gripper_controller/gripper_cmd (action).

This way the sliders actually command the simulated robot; /joint_states stays
from joint_state_broadcaster (real sim state) for TF and display.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration


ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
]
GRIPPER_JOINT = 'gripper_left_joint'


class JointStateGuiBridge(Node):
    def __init__(self):
        super().__init__('joint_state_gui_bridge')
        self.declare_parameter('arm_controller_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('trajectory_duration_sec', 0.2)

        arm_topic = self.get_parameter('arm_controller_topic').value
        gripper_action = self.get_parameter('gripper_action').value
        self._arm_pub = self.create_publisher(JointTrajectory, arm_topic, 10)
        self._gripper_client = ActionClient(self, GripperCommand, gripper_action)
        self._traj_duration = self.get_parameter('trajectory_duration_sec').value

        self._sub = self.create_subscription(
            JointState,
            'joint_states_desired',
            self._on_desired,
            10
        )
        self._last_gripper_pos = 0.0
        self.get_logger().info(
            'Joint state GUI bridge: forwarding joint_states_desired -> arm_controller + gripper_controller'
        )

    def _on_desired(self, msg: JointState):
        if not msg.name:
            return
        name_to_pos = dict(zip(msg.name, msg.position)) if msg.position else {}

        # Arm: build trajectory for the 5 joints
        arm_positions = []
        for j in ARM_JOINTS:
            if j in name_to_pos:
                arm_positions.append(name_to_pos[j])
            else:
                return  # need all arm joints
        if len(arm_positions) != len(ARM_JOINTS):
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = ''
        traj.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(sec=int(self._traj_duration), nanosec=int((self._traj_duration % 1) * 1e9))
        traj.points = [point]
        self._arm_pub.publish(traj)

        # Gripper: send action goal if we have gripper_left_joint
        if GRIPPER_JOINT in name_to_pos:
            pos = float(name_to_pos[GRIPPER_JOINT])
            if pos != self._last_gripper_pos:
                self._last_gripper_pos = pos
                self._send_gripper_goal(pos)

    def _send_gripper_goal(self, position: float):
        if not self._gripper_client.wait_for_server(timeout_sec=0.5):
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        self._gripper_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateGuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
