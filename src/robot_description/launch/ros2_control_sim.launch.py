"""
ros2 launch robot_description ros2_control_sim.launch.py

Launch file for ros2_control simulation with Gazebo Harmonic.

This launch file:
1. Starts Gazebo with the tabletop world
2. Spawns the robot from URDF (with ros2_control tags)
3. Loads and starts ros2_control controllers
4. Starts robot_state_publisher for TF
5. Bridges clock from Gazebo to ROS2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    
    # Get package share directory for file paths (must be resolved string for URDF/config)
    pkg_share = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    
    # Read URDF content for robot_state_publisher
    # Replace placeholder with actual controller config path
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    robot_description_content = robot_description_content.replace(
        'CONTROLLER_CONFIG_PATH', controllers_file
    )

    # Path to world file
    world_file = PathJoinSubstitution([
        pkg_robot_description, 'worlds', 'tabletop.sdf'
    ])

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find models
    models_path = os.path.join(pkg_share, 'models')
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

    # ==================== ROBOT STATE PUBLISHER ====================
    # Publishes TF transforms from URDF - needed for visualization and planning
    # Also provides robot_description parameter that gz_ros2_control reads
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    # ==================== SPAWN ROBOT ====================
    # Spawn robot in Gazebo from URDF (not SDF) - required for ros2_control
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arm_5dof',
            '-string', robot_description_content,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-allow_renaming', 'true',
        ],
        output='screen'
    )
    # Delay spawn so Gazebo world is fully loaded; avoids gz_ros2_control plugin init race
    delayed_spawn = TimerAction(
        period=4.0,
        actions=[spawn_robot],
    )

    # ==================== ROS2 CONTROL ====================
    # Spawn joint_state_broadcaster - publishes joint states from Gazebo to /joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Spawn arm_controller - trajectory controller for the 5-DOF arm
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Spawn gripper_controller - position controller for the gripper
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ==================== JOINT STATE GUI + BRIDGE ====================
    # GUI publishes to joint_states_desired (remapped) so it doesn't conflict with
    # joint_state_broadcaster on /joint_states. Bridge forwards desired -> arm + gripper controllers.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('joint_states', 'joint_states_desired')],
        parameters=[{'use_sim_time': True}],
    )
    # Run bridge from share path (works even if lib/ install is missing)
    bridge_script = os.path.join(
        get_package_share_directory('robot_description'),
        'scripts',
        'joint_state_gui_bridge.py',
    )
    joint_state_gui_bridge = ExecuteProcess(
        cmd=['python3', bridge_script, '--ros-args', '-p', 'use_sim_time:=true'],
        name='joint_state_gui_bridge',
        output='screen',
    )

    # ==================== BRIDGES ====================
    # Bridge for clock (Gazebo -> ROS2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ==================== EVENT HANDLERS ====================
    # Wait for spawn to complete and gz_ros2_control plugin to init before starting controllers
    delayed_controller_spawner = TimerAction(
        period=10.0,  # After delayed_spawn (4s) + time for spawn + plugin/controller_manager init
        actions=[
            joint_state_broadcaster_spawner,
        ]
    )

    # Chain controller spawning: arm after joint_state_broadcaster
    arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Chain controller spawning: gripper after arm
    gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
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
        # Robot (spawn after Gazebo world is ready)
        robot_state_publisher,
        delayed_spawn,
        # Sliders: GUI -> joint_states_desired; bridge -> arm_controller + gripper_controller
        joint_state_publisher_gui,
        joint_state_gui_bridge,
        # Bridges
        clock_bridge,
        # Controllers (delayed and chained)
        delayed_controller_spawner,
        arm_controller_event,
        gripper_controller_event,
    ])
