
# 22.02
study code in /src/openarm_bimanual_control

modify it:
for MoveIt–ros2_control–hardware/simulation to work, you need the scheme to run in real time and the position goal to be taken from an interface in a pre-defined data structure. The rest (planning, merging, sending trajectory to the controller) is what you already have; the missing piece for “from an interface” is to plug that interface and data structure into the node that sets the target (or builds the trajectory).


# openarm packages
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true

## openarm model rendering and controlling in rviz
1st terminal:
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true 

2nd terminal:
ros2 launch openarm_bimanual_control merge_execute.launch.py