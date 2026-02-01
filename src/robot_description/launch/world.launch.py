import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Path to world file
    world_file = PathJoinSubstitution([
        pkg_robot_description, 'worlds', 'tabletop.sdf'
    ])

    # Launch argument for world file (can override)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )

    # Launch Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items()
    )

    return LaunchDescription([
        world_arg,
        gazebo,
    ])
