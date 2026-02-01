import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package path
    pkg_robot_description = FindPackageShare('robot_description')

    # URDF file path
    urdf_file = os.path.join(
        FindPackageShare('robot_description').find('robot_description'),
        'urdf', 'robot.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    # Launch arguments for spawn position
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.8', description='Z position (on table)')

    # Robot State Publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arm_5dof',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen'
    )

    # Bridge for clock (Gazebo -> ROS2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
    ])
