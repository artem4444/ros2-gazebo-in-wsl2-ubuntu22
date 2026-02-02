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
- commit: controlled model of a robot in Gazebo with ros2_control via a bridge

1. control the ros2_control itself with higher level sequence of positional commands (moveit2-like) from a file (later this will be a topic with messages: for now- file with hardcoded timestamps)
- commit: coded a time-relative control of a robot in Gazebo



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



# control of the robotic model in Gazebo

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




# ros-humble-ros-gz bridge
sudo apt install ros-humble-ros-gz

ros_gz_bridge = Generic message translator between ROS2 ↔ Gazebo
Clock synchronization
Sensor data (cameras, lidar)
Simple commands (cmd_vel for mobile robots)
Does NOT handle joint control for arms

# gz_ros2_control plugin

ros2_control + gz_ros2_control = Joint control system
gz_ros2_control plugin runs inside Gazebo, interfaces with simulated joints
controller_manager loads controllers
Controllers (e.g., joint_trajectory_controller) receive commands and move joints
