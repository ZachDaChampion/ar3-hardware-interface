# ar3-hardware-interface

ROS 2 (Humble) hardware interface for the Annin Robotics AR3 using ros2_control

## Joint Names

The AR3 has 6 joints, each with both a `position` and `velocity` interface. These joints are named
as follows:

    - {$prefix}joint_0
    - {$prefix}joint_1
    - {$prefix}joint_2
    - {$prefix}joint_3
    - {$prefix}joint_4
    - {$prefix}joint_5

It also supports a `gripper` position interface, which is named:

    - {$prefix}gripper
