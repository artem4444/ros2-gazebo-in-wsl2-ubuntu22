from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_5dof", package_name="arm_5dof_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    # use_sim_time:=true when RViz is used with Gazebo so plugin accepts /joint_states and planning scene
    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # MotionPlanning plugin needs robot_description and robot_description_semantic;
    # generate_moveit_rviz_launch() does not pass them, so the robot model fails to load
    # when launching moveit_rviz.launch.py alone.
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )
    return ld
