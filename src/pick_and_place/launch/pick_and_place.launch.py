"""
Launch file for the pick and place demo.

This launch file starts:
1. The full ros2_control simulation (Gazebo + controllers)
2. MoveIt2 move_group (after a delay), which sends trajectories to
   /arm_controller/follow_joint_trajectory (configured in moveit_controllers.yaml)
3. The pick_and_place node (after a delay for startup)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_moveit_config = get_package_share_directory('arm_5dof_moveit_config')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'ros2_control_sim.launch.py')
        )
    )

    # MoveIt2 move_group: start after sim and controllers so it can send
    # trajectories to /arm_controller/follow_joint_trajectory
    # use_sim_time:=true required for Gazebo (/clock, /joint_states timestamps)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
        ),
        launch_arguments=[('use_sim_time', 'true')],
    )
    delayed_move_group = TimerAction(
        period=12.0,
        actions=[move_group_launch],
    )

    # Start pick_and_place node after delay (wait for controllers and move_group)
    pick_and_place_node = TimerAction(
        period=15.0,  # Wait 15 seconds for Gazebo + controllers to fully start
        actions=[
            Node(
                package='pick_and_place',
                executable='pick_and_place',
                name='pick_and_place_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        simulation_launch,
        delayed_move_group,
        pick_and_place_node,
    ])
