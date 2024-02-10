# ar3-hardware-interface

ROS 2 (Humble) hardware interface for the Annin Robotics AR3 using ros2_control

## Joint Names

The AR3 has 6 joints, each with both a `position` and `velocity` interface. These joints are named
as follows:

    - {$prefix}_joint_0
    - {$prefix}_joint_1
    - {$prefix}_joint_2
    - {$prefix}_joint_3
    - {$prefix}_joint_4
    - {$prefix}_joint_5

It also supports a `gripper` position interface, which is named:

    - {$prefix}_gripper
