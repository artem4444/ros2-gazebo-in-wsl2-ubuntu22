# Transform tree (TF)
transform tree (TF tree) is the graph of coordinate frames and how they relate to each other over time.
Nodes = coordinate frames (e.g. world, base_link, end_effector, camera_link).
Edges = transforms: rigid transformations (position + orientation) from one frame to another, published over time by various nodes.

# Joint Space, Joint variables
Joint space is the set of all possible configurations of the robot described by its joint variables. Each joint contributes one coordinate: angle (revolute) or displacement (prismatic). For a robot with n joints, joint space is an n-dimensional manifold (often a subset of R^n, e.g. T^n for n revolute joints). A point in joint space is a vector q = (q₁, …, qₙ); the forward kinematics map sends q to the end-effector pose in task (Cartesian) space.

*To fully describe the robot’s configuration in joint space, you use a joint variable vector with one scalar per joint*

Joint variables are the scalar quantities that define the relative configuration of two consecutive links in a kinematic chain. For a revolute joint, the joint variable is the angle θ (rotation about the joint axis). For a prismatic joint, it is the displacement d (translation along the joint axis). They are the generalized coordinates used to describe the configuration of the robot: for an n-joint serial manipulator, the joint vector is q = (q₁, …, qₙ), where each qᵢ is the variable of joint i. Joint variables form the coordinates of joint space.




