# RT control

frequency control of the is CAN network loop is done at ros2_control_node


The rate at which the hardware interface is called (read → controller update → write) is set in controller_manager and thus in ros2_control_node:
```yaml
# ros2_controllers.yaml (e.g. canopen_tests/config/robot_control/)
controller_manager:  
    ros__parameters:    
        update_rate: 100  # Hz  ← this is the ros2_control loop frequency
```
ros2_control_node runs a loop at update_rate (e.g. 100 Hz).
Each tick it:
    calls read() on the hardware interface (RobotSystem),
    updates all loaded controllers,
    calls write() on the hardware interface.
So the frequency of the “control loop” (read/write cycle) is done at ros2_control_node via update_rate.


# 2 arms
can0, can1, etc. are the Linux (SocketCAN) interface names for CAN controllers. 
They usually map to physical CAN ports:
    can0 = first CAN controller (e.g. first physical port)
    can1 = second CAN controller (e.g. second physical port)

For arm_left on can0 and arm_right on can1 you don’t split frames. You have two independent CAN buses and represent them as two separate ros2_control hardware interfaces (two RobotSystems):
Left arm: one <ros2_control> with RobotSystem, can_interface_name: can0, and a bus_config that only has the left-arm drives.
Right arm: another <ros2_control> with RobotSystem, can_interface_name: can1, and a different bus_config for the right-arm drives.
Each interface talks only to its own port; there is no “one frame divided into two” — each bus has its own frames.

controller_manager (one update loop)
    │
    ├─ read()/write() → RobotSystem (left)  → can0  → left-arm drives
    │
    └─ read()/write() → RobotSystem (right) → can1  → right-arm drives



# Phy USB CAN adapter
One USB CAN adapter = one CAN bus = one USB port

adapters have ready linux drivers

**In the hardware interface you still talk to CAN (the logical interface), not to the adapter driver**

Kernel: The adapter driver (e.g. peak_usb, gs_usb) is loaded when you plug the USB CAN adapter. It creates a SocketCAN interface (e.g. can0, can1). That’s a normal Linux network-style interface for CAN.
Your app (ros2_canopen): You only use the interface name (can0, can1). The stack opens that interface and sends/receives CAN frames via SocketCAN. It does not know or care whether can0 is built-in CAN or a USB adapter.
So in the hardware interface you set can_interface_name to that interface (e.g. can0). You’re setting up communication with that CAN interface, not “with the adapter driver” in your code.

*x2 arms, x2 CAN buses → x2 hardware interfaces*

*from hardware interface package perspective- it doesn't matter if there is a direct connection to CAN port, or a USB adapter with it's own Linux driver is invloved in the middle*



# ros2_control package
`ros2_control_demo_example_7.xml` is a pluginlib plugin description file
It tells ROS 2 which plugins this package provides and how to load them (library + class name)

*ros2_control hardware interface package is built around a small set of lifecycle and update hooks that you implement*

