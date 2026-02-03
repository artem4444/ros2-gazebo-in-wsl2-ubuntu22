"""
Demo launch for RViz-only: no ros2_control (no Gazebo).
Starts robot_state_publisher, joint_state_publisher_gui, move_group, and RViz.
The robot is visible and you can plan; Execute sends to a controller that is not
running (visual-only demo).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm_5dof", package_name="arm_5dof_moveit_config")
        .to_moveit_configs()
    )
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz")
    )

    # Static TF: world -> base_link (so RViz Fixed Frame "world" exists and planning scene can render)
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            output="log",
        )
    )

    # Static TFs for virtual joints (if any)
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Robot state publisher (publishes robot_description and TFs from joint_states)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py")),
        )
    )

    # Joint state publisher GUI – publishes /joint_states so the robot is visible in RViz
    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[moveit_config.robot_description],
        )
    )

    # Move group
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # RViz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    return ld
