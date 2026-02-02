import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    
    # Get package share directory for file paths
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    models_path = os.path.join(pkg_share, 'models')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Read URDF content for robot_state_publisher
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    # Path to world file
    world_file = PathJoinSubstitution([
        pkg_robot_description, 'worlds', 'tabletop.sdf'
    ])

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find our model
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.pathsep, models_path]
    )

    # ==================== LAUNCH ARGUMENTS ====================
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Robot X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Robot Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.4', description='Robot Z position (table surface)')

    # ==================== GAZEBO ====================
    # Launch Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items()
    )

    # ==================== ROBOT ====================
    # Robot State Publisher - publishes TF transforms from URDF (for RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # Spawn robot in Gazebo using SDF model
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arm_5dof',
            '-file', os.path.join(models_path, 'arm_5dof', 'model.sdf'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen'
    )

    # ==================== BRIDGES ====================
    # Bridge for clock (Gazebo -> ROS2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        # Environment
        gz_resource_path,
        # Arguments
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        # Gazebo
        gazebo,
        # Robot
        robot_state_publisher,
        spawn_robot,
        # Bridges
        clock_bridge,
    ])
