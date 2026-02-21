# perception sensors - moveit2 data flow

Physics
→ Analog signals
→ Digital packets
→ ROS messages
→ Time-synced data
→ Frame-transformed data
→ 3D geometry
→ Segmented clusters
→ Object pose estimates
→ Semantic labels
→ Planning scene geometry
→ Grasp candidates
→ IK solutions (joint space)
→ Planned trajectories
→ Controller setpoints
→ Motor torques
→ Motion
→ Updated world state
→ Symbolic task decisions


[ Sensors / Hardware ]
        ↓
ros2_control hardware interface
        ↓
JointState messages
        ↓
MoveIt RobotState
        ↓
MoveIt PlanningScene
        ↓
MoveIt Task Constructor
        ↓
Trajectory
        ↓
ros2_control controllers
        ↓
Hardware



# Octomap
render octomap in real time- to see what moveit2 sees from real world


# Moveit2 package: /config
