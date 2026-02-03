# MoveIt2 (ROS 2 Humble): Install and Test in RViz

Plan with commands to install MoveIt2 and test it in RViz.

---

## 1. Prerequisites

- ROS 2 Humble installed and sourced.

```bash
source /opt/ros/humble/setup.bash
```

---

## 2. Install MoveIt2 (binary)

```bash
sudo apt update
sudo apt install -y ros-humble-moveit
```

Optional: Install Panda demo packages so you can run a ready-made demo in RViz without creating your own config:

```bash
sudo apt install -y ros-humble-moveit-resources-panda-moveit-config
```

Optional but recommended (avoids known RMW issues): use Cyclone DDS:

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Test in RViz

### Option A: Panda demo (if you installed the panda config)

Starts RViz with the MotionPlanning plugin and a fake Panda robot. You can plan and visualize trajectories (no real/sim robot).

```bash
source /opt/ros/humble/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

In RViz:

- **Where the MotionPlanning UI is:** It’s a **Display**, not a Panel. Left side of RViz → **Displays** → **Add** → open **moveit_ros_visualization** → **MotionPlanning** → OK. Expand it to see **Planning** (Plan / Execute). If the plugin isn’t in the list: `sudo apt install -y ros-humble-moveit-ros-visualization`, then relaunch. Use the **Planning** tab there.
- Set a goal (drag the interactive marker or use “Update”).
- Click **Plan** then **Execute** (execution is fake; you only see the motion in RViz).

### Option B: MoveIt Setup Assistant (to prepare your own robot later)

Opens the GUI to create a MoveIt config from a URDF (e.g. from `robot_description`). Use this when you want to test MoveIt in RViz with your arm’s URDF.

```bash
source /opt/ros/humble/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

In the GUI: load your URDF (e.g. from `robot_description`), define planning groups, end-effector, self-collision, then save the config package. After building that package, you can run its `demo.launch.py` to get the same “plan in RViz” workflow as the Panda demo.

---

## 4. Test with your robot (robot_description) later

1. Run Setup Assistant (Option B above).
2. Load URDF: e.g. open `src/robot_description/urdf/robot.urdf` (or export from your launch if it uses xacro).
3. Create planning group(s): e.g. `arm` (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2) and optionally gripper; set end-effector link (e.g. `gripper_base_link` or the last link before fingers).
4. Generate the MoveIt config package and build it in your workspace.
5. Launch:  
   `ros2 launch <your_moveit_config_package> demo.launch.py`  
   This will start `move_group` + RViz with your robot; use the same Plan/Execute flow as the Panda demo.

---

## 5. Summary

| Step | Command / action |
|------|-------------------|
| Source Humble | `source /opt/ros/humble/setup.bash` |
| Install MoveIt2 | `sudo apt install -y ros-humble-moveit` |
| (Optional) Panda demo | `sudo apt install -y ros-humble-moveit-resources-panda-moveit-config` |
| (Optional) Cyclone DDS | `sudo apt install -y ros-humble-rmw-cyclonedds-cpp` and export `RMW_IMPLEMENTATION` |
| Test in RViz (Panda) | `ros2 launch moveit_resources_panda_moveit_config demo.launch.py` |
| Prepare your robot | `ros2 launch moveit_setup_assistant setup_assistant.launch.py` → load URDF, create config, then run that config’s `demo.launch.py` |

After this, MoveIt2 is installed and tested in RViz; you can then connect it to Gazebo and your `arm_controller` for pick-place.

---

## 6. Same setup with your URDF (robot_description)

To launch the same MoveIt + RViz demo (drag EE, IK, plan, execute) using your 5-DOF arm from `robot_description`, you need a **MoveIt config package** for that robot. Create it once with Setup Assistant, then launch its demo.

### 6.1 Create the MoveIt config package

**1. Start Setup Assistant**

```bash
cd /home/artem/ros2_ws
source /opt/ros/humble/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**2. Load your URDF**

- **File** → **Load File** (or "Load URDF").
- Open: **`/home/artem/ros2_ws/src/robot_description/urdf/robot.urdf`**
- If the loader complains (e.g. about `CONTROLLER_CONFIG_PATH` or Gazebo tags), the assistant may still accept it; if it fails, use a copy of the URDF with the `<ros2_control>` and `<gazebo>` blocks removed for this step only.

**3. Self-Collision (tab)**

- Leave defaults or click **Generate Collision Matrix**.
- **Next**.

**4. Virtual Joint (tab)**

- Usually **Skip** (robot is fixed to world).
- **Next**.

**5. Planning Groups (tab)**

- **Add** a group:
  - **Group name:** `arm`
  - **Kinematic Solver:** kdl_kinematics_plugin/KDLKinematicsPlugin (default)
  - **Add links:** Add the chain from `base_link` to the link you want as end-effector. Easiest: add **Links**: `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link` (or use "Add Chain" from `base_link` to `wrist_2_link`).
  - **Add joints:** `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`.
- **Save**, then **Next**.

**6. Robot Poses (tab)**

- **Add** a pose, e.g. name `home`, set joint values (e.g. all 0) so the arm is in a default pose.
- **Next**.

**7. End Effectors (tab)**

- **Add**:
  - **Name:** `gripper`
  - **Group:** `arm`
  - **Parent Link:** `wrist_2_link`
  - **Parent Group:** (leave empty)
- **Save**, **Next**.

**8. Passive Joints (tab)**

- Add `gripper_left_joint` and `gripper_right_joint` if you want them passive in planning (optional; you can skip).
- **Next**.

**9. Author Information (tab)**

- Fill as you like, **Next**.

**10. Configuration Files**

- **Browse** and set the path to your workspace **source** folder:  
  **`/home/artem/ros2_ws/src`**
- **Package name:** e.g. `arm_5dof_moveit_config` (or `robot_description_moveit_config`).
- Click **Generate Package**. Close the assistant when done.

### 6.2 Build and launch the demo

```bash
cd /home/artem/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select arm_5dof_moveit_config
source install/setup.bash
ros2 launch arm_5dof_moveit_config demo.launch.py
```

(Replace `arm_5dof_moveit_config` with the package name you chose.)

RViz will open with your robot. Add the **MotionPlanning** display if it’s not there (Displays → Add → moveit_ros_visualization → MotionPlanning). Set **Fixed Frame** to `base_link`, **Planning Group** to `arm`, enable **Query Goal State**, then drag the EE and use **Plan** / **Execute** as with the Panda.

### 6.3 If the demo expects a real controller

The generated config may point trajectory execution at a controller name (e.g. `arm_controller`). The **demo** launch usually starts a **fake** controller so it runs without Gazebo or hardware. If your demo launch fails with “controller not found”, open the generated config (e.g. `arm_5dof_moveit_config/config/moveit_controllers.yaml`) and ensure the demo uses a fake controller or the `fake_controller_manager`; the MoveIt Setup Assistant often generates this for the demo by default.
