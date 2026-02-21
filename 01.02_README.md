# Plan
in desktop room: from laptop

repo = modelling a robot morphology in Rviz

1. working wsl2 ubuntu22 instance: initialized a new one
2. install gazebo, rviz: use manual- compatible versions
    
    ‘’’ how to run ros2-ws with gazebo simulation of my custom robot?
    
3. init a new ros2-ws
4. URDFs to model a 2 arm robot with actuators as hollow-shaft Myactuator

continuing of project: controlling actuators with gazebo_virtual_hardware_interfaces

ros2_control connecting to this ros2-ws


## 01.02
RViz setup: last problem: visualize the model from the URDF: currently it doesn’t render

next session plan:

1. launch the robot_description package via launch file: ask to generate command
2. fix the issue with URDF not rendering
- commit: rendered robot_description with RViz2

1. redact URDF to describe a 5-DOF robotic arm with a grabber end-effector
2. manually change positions of joints in RViz: direct manual control 
- commit: direct control of Joints positions of a digital model of a 5-DOF robotic arm in RViz2

1. find a compatible with the stack Gazebo version: ask Perplexity
2. install Gazebo
3. connect Gazebo with the robot_description package
4. manage the mode in Gazebo 
5. add few mock objects to the Gazebo simulation: test interacting with them using the grabber
- commit: added 5-DOF robotic arm in Gazebo {version}, simulated it’s interacting with mock 3d objects


## 02.02

MOGI-ROS week 3-4 gazebo integration section

1. “”” what controller node we use to control robotic models inside the gazebo ? Is robotic arm control implemented right in the gazebo gui? is teleop_… ?
- commit: in Gazebo controlled the robotic arm's joints using Joint Position Controller gui plugin, replaced mock objects simulated it’s interacting with mock 3d objects




1. “”” how to control the robotic model inside the gazebo using ros2_control? via the ros-bridge?
2. implement the ros2_control of the gazebo model
- commit: controlled model of a robot in Gazebo with ros2_control



1. control the ros2_control itself with higher level sequence of positional commands (moveit2-like) from a file (later this will be a topic with messages: for now- file with hardcoded timestamps)
- commit: coded a time-relative control of a robot in Gazebo


## 03.02

- what the Python node controlled: trajectory of end-effector only? When it runs- What controls trajectories of each of the joints?
- find phd video: how we plan pick-place actions in ros2: vision, tactile sensors

1. chosen a high-level control framework
2. connected it to ros2_control using recommended communication pattern (actions/services/topics based)
- commit: the setup took {available level of abstraction} commands and completed control of robotic arm model doing a pick-place action on a mock object inside the gazebo simulation



## 10.02
- Nodes between moveit and data in frames, that is from perception sensors: tactile, lidar etc :: what these nodes are supposed to translate that data into, for moveit?

- ros2 actions list: where ros2_control gets high level commands from?
- When I connect moveit to ros2_control : I configure moveit to send those actions at a rate via the moveit config package?


ros2_control controllers take action and executes them, when joint_state_publisher_gui is not interrupting
/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/gripper_controller/gripper_cmd [control_msgs/action/GripperCommand]


## 11.02
1. defined moveit2 output format
2. integrated moveit2 output actions to ros2 controllers
- tested moveit2 - ros2 controllers control pipeline of the model in gazebo

1. “”” what data from sensors that’s is used in ros2 controllers? how it is used?
2. defined data structures, in which perception nodes must translate data for moveit2




# Testing a newly installed Ubuntu22

```bash
# 1. System
uname -a                    # Kernel OK
cat /etc/os-release         # Ubuntu 22.04 confirmed
whoami                      # User exists (not root)

# 2. Network
ping -c 2 google.com        # Internet works
curl -s https://packages.ros.org > /dev/null && echo "ROS repo reachable"

# 3. Package Manager
sudo apt update             # Repos accessible
sudo apt install -y curl gnupg lsb-release  # Can install packages

# 4. Locale
locale                      # UTF-8 set (ROS2 requirement)

# 5. Disk Space
df -h /                     # >10GB free recommended

# 6. Graphics (for Gazebo)
glxinfo | grep "renderer"   # GPU detected (WSL2: mesa/d3d12)
```

# wsl2 ubuntu22
## wsl2 ubuntu22 instance virtual hard disk moving to disk D
```bash
# Export the distro
wsl --export Ubuntu-22.04 D:\ubuntu22-backup.tar

# Unregister from C:
wsl --unregister Ubuntu-22.04

# Import to D: drive
wsl --import Ubuntu-22.04 D:\WSL\Ubuntu-22.04 D:\ubuntu22-backup.tar

# Set default user (after import, it defaults to root)
ubuntu2204.exe config --default-user artem
```

## wsl2 files system
i can access windows disks from it

`cd ~` to enter /home


# wsl2 GUI Forwarding
WSL2 version has WSLg built-in, which provides native GUI support without needing VcXsrv. The X11 socket is already present.

X11 (also called X Window System or simply X) is a windowing system protocol that provides the basic framework for building graphical user interfaces (GUIs) on Unix-like operating systems, including Linux.
    X11 = protocol for displaying graphics on Unix/Linux
    X Server = runs locally, draws pixels on screen
    X Client = your application, sends drawing commands
    Network-aware = can display remote apps locally (great for SSH + GUI)
    Still widely used despite Wayland being the newer replacement

# ros2-ws installation order

1. ROS2 Humble          ← Install first
    test rviz gui
2. Gazebo + ROS2 packages
3. Create ros2_ws
4. Create my_robot_control package
5. Add URDF, config, launch files
6. Build & Run


# sourcing
source ~/ros2_ws/install/setup.bash

source /opt/ros/humble/setup.bash

we source so that:
-shell would know where ROS2 tools are (ros2, colcon)
-any process we launch inherits paths to find ROS2 libraries/packages
-Without sourcing, commands like ros2 run won't even be found

If you open a new terminal, you need to source again because it's a fresh shell with no ROS2 environment variables set.

we need to source new packages after running colcon build


# new package in ros2 ws


# build the ws with colcon


# ros2 installations, dependencies management

```bash
# 1. One-time setup (tools)
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update

# 2. Per-workspace (dependencies)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build
```
`rosdep install --from-paths src --ignore-src -r -y`

rosdep = ROS Dependency Manager

package.xml (declares dependencies)  →  rosdep (installs them)
                                     →  colcon (uses them during build)


# robot_description package
1. Create package: robot_description
2. Fill with: URDF, meshes, rviz config, launch file
3. Build workspace: colcon build
4. Launch: ros2 launch robot_description display.launch.py
    (this starts RViz2 with your robot loaded)


# ros2 launches
It's a convention, not a requirement. Common naming patterns:
Name	Purpose
display.launch.py	Visualize robot in RViz (no simulation)
gazebo.launch.py	Launch Gazebo simulation
bringup.launch.py	Start real robot hardware
robot.launch.py	Generic main launch
spawn.launch.py	Spawn robot into Gazebo

bringup.launch.py typically chains/includes launch files from multiple packages

## testing launch files separately

example of 2 launch files integration

world.launch.py
      │
      ▼
┌─────────────────────────────┐
│   Gazebo Server (gz sim)    │  ← Single running instance
│   - Loads tabletop.sdf      │
│   - Physics engine running  │
│   - Listening for requests  │
└─────────────────────────────┘
      ▲
      │
spawn_robot.launch.py
      │
      ▼
┌─────────────────────────────┐
│   ros_gz_sim create node    │
│   - Connects to gz server   │
│   - Sends "spawn model"     │
│     request via gz-transport│
└─────────────────────────────┘



## unified launches into simulation.launch.py
```bash
# Rebuild to install the new launch file
cd ~/ros2_ws && colcon build --packages-select robot_description

# Source and launch
source install/setup.bash
ros2 launch robot_description simulation.launch.py
```


# RViz2
RViz2 = Visualization Tool for Robot Descriptions

URDF (robot model)  →  robot_state_publisher  →  RViz2 (renders it)


# nodes, topics, messages
in ROS2 each topic has exactly one message type

```bash
ros2 topic list -t
ros2 node list
rqt_graph

```



# Gazebo Harmonic installation
APT (Advanced Package Tool) is a package management system used by Debian-based Linux distributions

apt update refreshes the local package index — it downloads the latest list of available packages and their versions from the repositories configured on your system

gnupg (GNU Privacy Guard)
Encryption and signing tool — used to verify package authenticity

lsb-release
Linux Standard Base release tool — provides information about your Linux distribution.

```bash
# Install Gazebo Harmonic in WSL and Windows (Ubuntu 24.04 (22 in my case))
# To install Gazebo, we need to perform the following steps. In the Ubuntu 24.04 WSL terminal type:

sudo apt-get update
sudo apt-get install lsb-release gnupg
# Then type

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
# This should install Gazebo Harmonic

# Run Gazebo Simulations in Windows through WSL
# To run the simulations, we need to type

gz sim shapes.sdf

```

# launch Gazebo world

```bash
cd ~/ros2_ws
colcon build --packages-select robot_description
source install/setup.bash

# Restart Gazebo and spawn
ros2 launch robot_description world.launch.py
# (new terminal)
ros2 launch robot_description spawn_robot.launch.py

```



# GUI plugin based control of the robotic model in Gazebo

User Input (GUI/script/MoveIt2)
    Joint Position Control gui plugin (in 3-dots menu)

┌─────────────────────────────────────────────────────────────┐
│                     GAZEBO HARMONIC                          │
│                                                              │
│   GUI JointControl Plugin (sliders)                          │
│            │                                                 │
│            ▼ (gz-transport)                                  │
│   JointPositionController plugins (PID per joint)            │
│            │                                                 │
│            ▼                                                 │
│   Gazebo Physics Engine → Joints move                        │
│                                                              │
│   JointStatePublisher plugin → publishes joint states        │
└─────────────────────────────────────────────────────────────┘


# ros2_control based control of the robotic model in Gazebo

## gz_ros2_control plugin
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS2 SIDE                                   │
│                                                                     │
│   Your Code / MoveIt2 / CLI                                         │
│         │                                                           │
│         ▼  (publishes to /joint_trajectory_controller/...)          │
│   joint_trajectory_controller                                       │
│         │                                                           │
│         ▼                                                           │
│   controller_manager (manages all controllers)                      │
│         │                                                           │
│         ▼  (hardware_interface API)                                 │
├─────────────────────────────────────────────────────────────────────┤
│                         GAZEBO SIDE                                 │
│                                                                     │
│   gz_ros2_control plugin (loaded inside Gazebo)                     │
│         │                                                           │
│         ▼  (applies forces/positions to joints)                     │
│   Gazebo Physics Engine                                             │
└─────────────────────────────────────────────────────────────────────┘


# ros2_control simulation

```bash
# Install ros2_control dependencies
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control

# Build and launch
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robot_description
source install/setup.bash
ros2 launch robot_description ros2_control_sim.launch.py
```

## ros2_control commands

```bash
# List controllers and their state
ros2 control list_controllers

# List hardware interfaces
ros2 control list_hardware_interfaces

# Check controller_manager status
ros2 control list_controller_types

# Send trajectory command to arm (move to position)
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint],
      points: [
        {positions: [0.5, 0.3, -0.5, 0.2, 0.0], time_from_start: {sec: 2}}
      ]
    }
  }"

# Control gripper (open/close)
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{
    command: {position: 0.02, max_effort: 10.0}
  }"

# Topic-based control (alternative to action)
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]
  }"
```


# Monitoring
*monitor all data flowing through the system using ROS2 CLI tool*
```bash
# List all topics
ros2 topic list

# List topics with their message types
ros2 topic list -t

# See publishing frequency of a topic
ros2 topic hz /joint_states

# See the actual data being published
ros2 topic echo /joint_states

# See bandwidth usage
ros2 topic bw /joint_states

# Get info about a topic (publishers, subscribers, type)
ros2 topic info /joint_states -v
```

## Troubleshooting: position commands not executing in Gazebo

If `ros2 action send_goal /arm_controller/follow_joint_trajectory ...` is accepted but the model in Gazebo does not move:

1. **Sim time**: The controller_manager and arm_controller must use simulation time. In `config/ros2_controllers.yaml` ensure `use_sim_time: true` is set under both `controller_manager.ros__parameters` and `arm_controller.ros__parameters`. Without this, trajectory timing does not follow `/clock` and the trajectory may never run.

2. **Do not cancel**: Let the goal run for the full trajectory duration (e.g. 2 seconds). Pressing Ctrl+C cancels the goal and stops execution.

3. **Verify `/clock`**: Run `ros2 topic hz /clock` while Gazebo is running; it should publish. The launch file bridges Gazebo clock to ROS 2.

4. **Verify joint state updates**: While sending a goal, run `ros2 topic echo /joint_states`. If arm joint positions do not change, commands are not reaching the hardware interface (check controller activation and config path). If they change but the model does not move in Gazebo, the issue is in gz_ros2_control or the simulation.

5. **Controller config path**: The launch file passes the full path to `ros2_controllers.yaml` into the URDF so the gz_ros2_control plugin can load it. The launch uses `get_package_share_directory('robot_description')` so the path resolves correctly from the install space.

6. **Launch timing**: The spawn is delayed 4 s so the Gazebo world is ready; controllers are started 10 s after launch. Wait until you see the arm_controller and gripper_controller reported as **active** (e.g. `ros2 control list_controllers`) before sending action goals. Sending a goal before the controller_manager and gz_ros2_control plugin are ready can result in accepted goals that never move the robot.

## topics of ros2_control control of model in gazebo
`artem@LAPTOP-15QP4R2F:~/ros2_ws$ ros2 topic list`
/arm_controller/controller_state
/arm_controller/joint_trajectory
/arm_controller/state
/arm_controller/transition_event
/clock
/dynamic_joint_states
/gripper_controller/transition_event
/joint_state_broadcaster/transition_event
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static


# ROS2 Communication Patterns
                Is it a long-running task?
                        │
            ┌────────────┴────────────┐
            │ YES                     │ NO
            ▼                         ▼
        Use ACTION              Need response?
(motion, navigation)              │
                        ┌─────────┴─────────┐
                        │ YES               │ NO
                        ▼                   ▼
                    Use SERVICE          Use TOPIC
                (config, query)     (sensors, state)


# Action-based control

## Actions in ROS2 Midleware
An action is actually built from 3 services + 2 topics:
/arm_controller/follow_joint_trajectory/
├── _action/send_goal        (Service)  - Send goal
├── _action/cancel_goal      (Service)  - Cancel request  
├── _action/get_result       (Service)  - Get final result
├── _action/feedback         (Topic)    - Progress updates
└── _action/status           (Topic)    - Goal status

why we can `ros2 action list`
actions still work at some frequency: and topics are mantained even between those actions are sent
┌─────────────────────────────────────────────────────────────────┐
│              ACTION SERVER (always running)                      │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  /_action/status topic    →  publishes at ~1 Hz         │    │
│  │  (always active, even with no goals)                    │    │
│  │                                                          │    │
│  │  Status: "No active goals" / "Goal executing" / etc.    │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  /_action/feedback topic  →  publishes during execution │    │
│  │  (10-50 Hz while goal is active)                        │    │
│  │                                                          │    │
│  │  "Current position: [0.2, 0.3, ...]"                    │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Services: send_goal, cancel_goal, get_result           │    │
│  │  (always available, waiting for requests)               │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘


## example of Action CLI command
this command contains everything needed for an action and if we want to implement some complex logic: we just need to code a node that will generate such commands and send them at pre-defined frequency 

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint],
      points: [
        {positions: [0.5, 0.3, -0.5, 0.2, 0.0], time_from_start: {sec: 2}}
      ]
    }
  }"
```

Your CLI command
    │
    ▼
ros2 action send_goal /arm_controller/follow_joint_trajectory ...
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│  arm_controller                                             │
│  (type: joint_trajectory_controller/JointTrajectoryController)
│                                                             │
│  Action Server: /arm_controller/follow_joint_trajectory     │
│  Action Type:   control_msgs/action/FollowJointTrajectory   │
└─────────────────────────────────────────────────────────────┘





# moveit2 installation
  moveit2 is a frameworks with many nodes, topics etc

installing moveit2 and testing it in rviz with urdf from robot_description

If you add MoveIt2 packages as dependencies in your package.xml, rosdep will resolve and install them

## testing moveit2 in rviz2 with panda config (standard robotic arm)
displays - MotionPlanning - Planning - Planning group

the point of MotionPlanning tool is to calculate IK for the position of the EE i defined by manually dragging it with the cursor, and to render new joint positions in the rviz gui


# moveit2 config
*MoveIt2 config package holds everything MoveIt needs to plan and (optionally) execute for your robot: robot model, groups, limits, planners, and controllers. It does not replace your URDF; it adds semantic and planning data on top of it.*

```
arm_5dof_moveit_config/
├── .setup_assistant          # Used by Setup Assistant to know URDF/SRDF paths
├── config/                   # YAML and RViz configs
│   ├── arm_5dof.srdf         # Semantic: groups, EE, collisions, poses
│   ├── joint_limits.yaml      # Overrides / extra limits for planning
│   ├── kinematics.yaml       # IK solver (e.g. KDL) per group
│   ├── moveit_controllers.yaml  # Controllers MoveIt uses for execution
│   ├── moveit.rviz           # RViz layout + MotionPlanning display
│   ├── ompl_planning.yaml    # (if present) OMPL planner tuning
│   └── pilz_cartesian_limits.yaml  # Pilz LIN/PTP/CIRC limits
└── launch/                   # Launch files
    ├── demo.launch.py        # RViz demo (no real hardware)
    ├── move_group.launch.py  # Starts move_group node
    ├── moveit_rviz.launch.py # RViz with MoveIt config
    ├── rsp.launch.py         # robot_state_publisher
    ├── spawn_controllers.launch.py  # Spawn ros2_control controllers
    └── ...
```

## moveit2 config package for 5-DOF arm form robot_description

Moveit Setup Assistent gui tool

```bash
cd /home/artem/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_5dof_moveit_config demo.launch.py
```



# *moveit2 connection to ros2_control*

moveit2 to ros2_control connection is similar in gz_ros2_control implementation and in real deployment, where ros2_contorl control real hardware via hardware_interface plugin

**In production, the MoveIt2 → ros2_control connection should be action-based**
format: yaml string 


*Broadcasters = ros2_control controllers that only read state (sensor) interfaces and publish them — same idea as joint_state_broadcaster, but for other sensors.*
Broadcasters (read state → publish topics)
Controller	State it reads	Publishes
joint_state_broadcaster	Joint position, velocity, effort	sensor_msgs/msg/JointState → /joint_states
ForceTorqueBroadcaster	Force/torque at a joint/link	geometry_msgs/msg/WrenchStamped (or similar)
IMUSensorBroadcaster	Orientation, angular velocity, linear acceleration	sensor_msgs/msg/Imu
RangeSensorBroadcaster	Range (e.g. sonar, ToF)	sensor_msgs/msg/Range
CartesianStateBroadcaster	Cartesian pose/velocity (e.g. from model/FK)	Cartesian state topic


## command ros2 controllers with moveit2 from rviz2 planning display

Rebuild:
   cd ~/ros2_ws && colcon build --packages-select arm_5dof_moveit_config pick_and_place   source install/setup.bash
Terminal 1 – Sim + move_group:
   ros2 launch pick_and_place pick_and_place.launch.py
Wait until Gazebo and the robot are up (~15 s).
Terminal 2 – RViz with sim time:
   ros2 launch arm_5dof_moveit_config moveit_rviz.launch.py use_sim_time:=true
In RViz: set Fixed Frame to world or base_link, set a goal with the MotionPlanning interactive marker, then click Plan and Execute. The arm in Gazebo should move.
If Plan or Execute still fail, check:
Fixed Frame in RViz matches a frame from your sim (e.g. world or base_link).
/joint_states is published: ros2 topic echo /joint_states --once
/move_action exists: ros2 action list | grep move_action
/arm_controller/follow_joint_trajectory exists: ros2 action list | grep follow_joint_trajectory


# high-level control framework
pick and place using moveit2 with perception 

## How to send a MoveIt goal

1. **RViz (MotionPlanning plugin)**  
   Launch sim + MoveIt2, open RViz with the MoveIt MotionPlanning panel. Set the goal with the interactive marker, then click **Plan** and **Execute**. MoveIt sends goals to `/move_action` and trajectories to `/arm_controller/follow_joint_trajectory`.

2. **Python (joint-space goal)**  
   Use the example script that sends a `MoveGroup` goal to `/move_action`:
   ```bash
   # With sim and move_group already running (e.g. ros2 launch pick_and_place pick_and_place.launch.py)
   ros2 run pick_and_place moveit_send_goal_example                    # default: home pose
   ros2 run pick_and_place moveit_send_goal_example -- 0.0 0.5 -0.8 0.3 0.0   # custom joint positions (rad)
   ```
   Joint order: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`.

3. **From your own node**  
   Create an `ActionClient` for `moveit_msgs/action/MoveGroup` with action name `/move_action`. Build a `MoveGroup.Goal` with `request.group_name = "arm"` and `request.goal_constraints` set to joint constraints (see `pick_and_place/moveit_send_goal_example.py`).


ready manuals:
- Automatic Addison channel




# moveit2 with teleoperating package
if i use teleoperation package: each time i send a teleoperation signal to moveit2: the previous set of joint trajectories gets interrupted and a new one is generated and passed to ros2 controllers by the moveit2 



# moveit2 perception
we publish messages with values retrieved by hardware interface from CAN frame
rate = frequnecy of CAN network
content ~= octomap required data






# moveit2 - isaac sim
we subsribe a node from Isaac Sim ActionGraph to moveit2 topic 

we control moveit2 content from rviz2 using Planning Display (we set trajectories for several Planning Grouos to execute simultaneously)


we can connect Unreal Engine the same way 


## accessing ros2 topic from outside ros2 
Isaac Sim has built‑in ROS 2 support (nodes, topics, actions). A node in the Isaac Sim ActionGraph can subscribe/publish directly to MoveIt2.
Unreal does not speak ROS 2 by default. You need a ROS 2 bridge (plugin or separate process) so that Unreal can publish/subscribe to the same ROS 2 topics/actions that MoveIt2 uses.