# MoveIt2 with Custom Robot — Fast Setup (No-Error Checklist)

Minimal guide to get a MoveIt2 config running with your URDF in RViz. Covers only what prevents common failures.

---

## 1. Prerequisites

- ROS 2 Humble, `source /opt/ros/humble/setup.bash`
- MoveIt2: `sudo apt install -y ros-humble-moveit`
- Your robot URDF in a package (e.g. `robot_description`)

---

## 2. Create config with Setup Assistant

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

- **Load URDF:** File → Load → e.g. `src/robot_description/urdf/robot.urdf`
- **Self-Collision:** Generate Collision Matrix → Next
- **Virtual Joint:** Skip → Next
- **Planning Groups:** Add group (e.g. `arm`). Add the arm joints (or use “Add Chain” from base link to tip link, e.g. `base_link` → `wrist_2_link`). **Save** → Next
- **Robot Poses:** Add e.g. `home` (joint values) → Next
- **End Effectors:** Add name (e.g. `gripper`), parent link (e.g. `wrist_2_link`), group `arm`. **Do not set `parent_group` to the same as `group`** (or leave parent_group empty). **Save** → Next
- **Passive Joints:** Optional → Next
- **Author:** Fill or leave (will need valid email in package.xml) → Next
- **Configuration Files:** Browse to `src/`, set package name (e.g. `my_robot_moveit_config`) → **Generate Package**

If Setup Assistant crashes on Load, try a URDF without `<ros2_control>` / `<gazebo>` blocks for this step only.

---

## 3. Fix generated package (avoid build/run errors)

### 3.1 `package.xml` — valid maintainer

Build fails with “Invalid email” / “Maintainers must have an email address” if left empty.

**Fix:** In `src/<pkg>/package.xml` set:

```xml
<maintainer email="user@todo.todo">YourName</maintainer>
<author email="user@todo.todo">YourName</author>
```

### 3.2 `config/joint_limits.yaml` — use floats

MoveIt expects **double** for velocity/acceleration. Integer values cause:

`parameter '...max_velocity' has invalid type: expected [double] got [integer]`

**Fix:** Use decimal form: `max_velocity: 1.0`, `max_acceleration: 0.0` (not `1` or `0`).

### 3.3 `config/kinematics.yaml` — define IK solver

Empty or missing kinematics causes “No kinematics plugins defined” and no interactive markers.

**Fix:** Add (replace `arm` with your group name):

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

### 3.4 `config/arm_5dof.srdf` (or your robot name) — end effector parent

Error: “Group 'arm' for end-effector 'gripper' cannot be its own parent”.

**Fix:** In the `<end_effector>` tag, **remove** `parent_group="arm"` (or do not set parent_group to the same as `group`). Example:

```xml
<end_effector name="gripper" parent_link="wrist_2_link" group="arm"/>
```

### 3.5 `config/moveit_controllers.yaml` — create if missing

Without any `*_controllers.yaml`, trajectory_execution is incomplete and the planning scene may not load correctly.

**Fix:** Create `config/moveit_controllers.yaml`:

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01
  trajectory_duration_monitoring: true

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      # ... all arm joints
```

(Adjust `controller_names` and `joints` to match your robot.)

---

## 4. Demo without Gazebo (RViz only)

If your URDF uses `gz_ros2_control`/Gazebo hardware, the **default** demo launch will start `ros2_control_node` and crash when no Gazebo is running.

**Fix:** Use a **custom demo** that does **not** start `ros2_control_node` and **does** start:

- `robot_state_publisher`
- `joint_state_publisher_gui` (so `/joint_states` exists and the robot is visible)
- Static TF `world` → `base_link` (so RViz Fixed Frame exists)
- `move_group`
- RViz

Replace `launch/demo.launch.py` with a launch that includes the above and **omits** `ros2_control_node` and spawn_controllers. Pass `moveit_config.robot_description` to `joint_state_publisher_gui`.

### RViz

- **Fixed Frame:** e.g. `world` (must be published; static_transform_publisher world → base_link).
- **MotionPlanning** display: Planning Scene Topic = `/monitored_planning_scene`, Trajectory Topic = `/display_planned_path`.

---

## 5. Build and run

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_moveit_config
source install/setup.bash
ros2 launch my_robot_moveit_config demo.launch.py
```

- Use **Planning Group** = your group (e.g. `arm`), enable **Query Goal State**, then **Plan** / **Execute**.

---

## 6. Quick checklist before first run

| Check | Purpose |
|-------|--------|
| `package.xml` maintainer/author have non-empty email | Avoid build failure |
| `joint_limits.yaml` uses `1.0` / `0.0` not `1` / `0` | Avoid move_group parameter type error |
| `kinematics.yaml` has group and KDL (or other) solver | IK and interactive markers work |
| SRDF end_effector has no `parent_group` = group | Avoid “cannot be its own parent” |
| `config/moveit_controllers.yaml` exists | Planning scene and trajectory execution load |
| Demo launch does not start ros2_control with Gazebo HW | Avoid crash when not running Gazebo |
| Static TF world → base_link in demo | “No planning scene” / fixed frame in RViz |
| RViz topics: `/monitored_planning_scene`, `/display_planned_path` | MotionPlanning display works |

---

## 7. Reference: minimal custom demo.launch.py (no ros2_control)

- Include: static TF (world → base_link), `rsp.launch.py`, `joint_state_publisher_gui` (with `moveit_config.robot_description`), `move_group.launch.py`, `moveit_rviz.launch.py`.
- Do **not** include: `ros2_control_node`, `spawn_controllers.launch.py`.
- Use `DeclareLaunchArgument` (not `DeclareBooleanLaunchArgument` from `launch.actions` in Humble) for args like `use_rviz`.

This gives a working RViz-only demo; Execute will not drive real hardware unless you later connect a real controller.
