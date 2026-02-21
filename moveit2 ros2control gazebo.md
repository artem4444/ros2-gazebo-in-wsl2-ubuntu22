# Planning Groups
usual and recommended setup is:
One planning group per arm (e.g. left_arm, right_arm).
One group per “hand”/effector if you want to plan gripper motions separately (e.g. left_gripper, right_gripper), or you can include the gripper joints in the arm group.


# Synchronization of Planning Groups actions
lan each group as usual, then in your controlling node merge the results into one JointTrajectory (or JointTrajectoryPoint) that contains:
all joint names (e.g. left_arm + right_arm + grippers),
one time axis (e.g. shared time_from_start),
for each time step, positions (and optionally velocities/accelerations) for every joint.
Send this single trajectory to one controller that owns all those joints.