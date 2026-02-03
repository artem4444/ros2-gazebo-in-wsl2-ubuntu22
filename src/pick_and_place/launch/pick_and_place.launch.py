"""
Launch file for the pick and place demo.

This launch file starts:
1. The full ros2_control simulation (Gazebo + controllers)
2. The pick_and_place node (after a delay for startup)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find packages
    pkg_robot_description = FindPackageShare('robot_description').find('robot_description')
    
    # Include the ros2_control simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'ros2_control_sim.launch.py')
        )
    )
    
    # Start pick_and_place node after delay (wait for controllers to load)
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
        pick_and_place_node,
    ])
