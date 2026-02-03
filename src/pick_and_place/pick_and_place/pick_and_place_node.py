#!/usr/bin/env python3
"""
Pick and Place Node - High-level control for robotic arm pick-and-place tasks.

This node demonstrates how to build high-level robot behaviors on top of ros2_control.
It sends action goals to the arm_controller and gripper_controller to perform:
1. Move to pre-grasp position
2. Approach object
3. Close gripper (grasp)
4. Lift object
5. Move to place position
6. Lower object
7. Open gripper (release)
8. Retreat

Architecture:
    PickAndPlaceNode  ──►  /arm_controller/follow_joint_trajectory (Action)
                      ──►  /gripper_controller/gripper_cmd (Action)
"""

"""
The node does control trajectories for each arm joint, in joint space.

If a joint target is beyond limits, behavior depends on the controller 
(e.g. JointTrajectoryController): 
it may clamp, reject, or fault. The node doesn’t check limits itself.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time


class PickAndPlaceNode(Node):
    """High-level pick and place controller using ros2_control actions."""

    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Joint names (must match your URDF/controller config)
        self.arm_joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
        ]
        
        # Action clients for arm and gripper
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )
        
        # ============== PREDEFINED POSITIONS ==============
        # These would typically come from perception/planning (e.g., MoveIt2)
        # Values are in radians for revolute joints
        
        self.positions = {
            # Home/safe position
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            
            # Pre-grasp: above the object
            'pre_grasp': [0.0, 0.5, -0.8, 0.3, 0.0],
            
            # Grasp: at the object (lower)
            'grasp': [0.0, 0.7, -0.5, -0.2, 0.0],
            
            # Lift: object lifted
            'lift': [0.0, 0.4, -0.6, 0.2, 0.0],
            
            # Pre-place: above place location (rotated 90 degrees)
            'pre_place': [1.57, 0.4, -0.6, 0.2, 0.0],
            
            # Place: at place location (lower)
            'place': [1.57, 0.7, -0.5, -0.2, 0.0],
            
            # Retreat: back up after placing
            'retreat': [1.57, 0.4, -0.6, 0.2, 0.0],
        }
        
        # Gripper positions (meters for prismatic joint)
        self.gripper_open = 0.025    # Open position
        self.gripper_closed = 0.003  # Closed/grasp position
        
        self.get_logger().info('Pick and Place Node initialized')

    def wait_for_servers(self, timeout_sec=10.0):
        """Wait for all action servers to be available."""
        self.get_logger().info('Waiting for action servers...')
        
        self.get_logger().info('  Waiting for arm controller...')
        if not self.arm_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('  Arm controller not available!')
            return False
        self.get_logger().info('  Arm controller: OK')
            
        self.get_logger().info('  Waiting for gripper controller...')
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('  Gripper controller not available!')
            return False
        self.get_logger().info('  Gripper controller: OK')
            
        self.get_logger().info('All action servers ready!')
        return True

    def move_arm(self, position_name: str, duration_sec: float = 2.0) -> bool:
        """
        Move arm to a named position (blocking call).
        
        Args:
            position_name: Key from self.positions dict
            duration_sec: Time to complete the motion
            
        Returns:
            True if motion succeeded, False otherwise
        """
        if position_name not in self.positions:
            self.get_logger().error(f'Unknown position: {position_name}')
            return False
            
        positions = self.positions[position_name]
        self.get_logger().info(f'Moving arm to: {position_name} {positions}')
        
        # Build trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names
        
        # Single point trajectory (controller interpolates)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)  # Zero velocity at end
        point.time_from_start = Duration(
            sec=int(duration_sec), 
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points = [point]
        
        # Send goal
        send_goal_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('  Goal rejected!')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'  ✓ Motion complete: {position_name}')
            return True
        else:
            self.get_logger().error(f'  ✗ Motion failed with status: {result.status}')
            return False

    def move_gripper(self, position: float, max_effort: float = 10.0) -> bool:
        """
        Move gripper to specified position (blocking call).
        
        Args:
            position: Target position in meters (0 = closed, 0.03 = fully open)
            max_effort: Maximum force in Newtons
            
        Returns:
            True if motion succeeded
        """
        action = 'Opening' if position > 0.01 else 'Closing'
        self.get_logger().info(f'{action} gripper to {position:.3f}m')
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        
        # Send goal
        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('  Gripper goal rejected!')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        self.get_logger().info(f'  ✓ Gripper at {result.result.position:.3f}m')
        return True

    def pick_and_place(self) -> bool:
        """
        Execute complete pick and place sequence.
        
        Returns:
            True if sequence completed successfully
        """
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('       PICK AND PLACE SEQUENCE STARTING')
        self.get_logger().info('='*60)
        
        # Check servers are available
        if not self.wait_for_servers():
            return False
        
        steps = [
            ('Step 1/9: Home position + Open gripper', 
             lambda: self.move_arm('home', 2.0) and self.move_gripper(self.gripper_open)),
            
            ('Step 2/9: Move to pre-grasp', 
             lambda: self.move_arm('pre_grasp', 2.0)),
            
            ('Step 3/9: Approach object', 
             lambda: self.move_arm('grasp', 1.5)),
            
            ('Step 4/9: Close gripper (GRASP)', 
             lambda: self.move_gripper(self.gripper_closed, max_effort=15.0)),
            
            ('Step 5/9: Lift object', 
             lambda: self.move_arm('lift', 1.5)),
            
            ('Step 6/9: Move to place location', 
             lambda: self.move_arm('pre_place', 2.5)),
            
            ('Step 7/9: Lower object', 
             lambda: self.move_arm('place', 1.5)),
            
            ('Step 8/9: Open gripper (RELEASE)', 
             lambda: self.move_gripper(self.gripper_open)),
            
            ('Step 9/9: Retreat and go home', 
             lambda: self.move_arm('retreat', 1.5) and self.move_arm('home', 2.0)),
        ]
        
        for step_name, step_func in steps:
            self.get_logger().info('')
            self.get_logger().info(f'>>> {step_name}')
            
            if not step_func():
                self.get_logger().error(f'FAILED at: {step_name}')
                return False
            
            time.sleep(0.3)  # Brief pause between steps
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('       PICK AND PLACE COMPLETE!')
        self.get_logger().info('='*60)
        return True


def main():
    """Entry point."""
    rclpy.init()
    
    node = PickAndPlaceNode()
    
    try:
        success = node.pick_and_place()
        if success:
            node.get_logger().info('Pick and place succeeded!')
        else:
            node.get_logger().error('Pick and place failed!')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
