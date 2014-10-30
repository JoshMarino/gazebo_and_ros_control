ME 495: Mini-Project - Gazebo and ROS Control
=============================================

Josh Marino and Mahdieh Javaremi
--------------------------------


## Project Documentation ##


#### Project Overview ####
Determine the relationship between Gazebo, ROS, ROS control, and ROS controllers. This will be implemented using the RRBot model.


#### Useful Tutorials ####
We followed three main tutorials in order to understand how the pieces fit together. The first tutorial on "Using a URDF in Gazebo" described the required and optional sections of a URDF.

1. [Using a URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf")
2. [Using Gazebo plugins with ROS](http://gazebosim.org/tutorials?tut=ros_gzplugins)
3. [ROS Control](http://gazebosim.org/tutorials/?tut=ros_control)



#### Tutorial Changes for ROS Indigo ####
The only change we found from following the third tutorial on ROS Control, was the following in `rrbot.xacro` lines 3-5 and 14-16, denoted with ***change***.

```
1)  <transmission name="tran1">
2)    <type>transmission_interface/SimpleTransmission</type>
3)    ***<joint name="joint1">***
4)      ***<hardwareInterface>EffortJointInterface</hardwareInterface>***
5)    ***</joint>***
6)    <actuator name="motor1">
7)      <hardwareInterface>EffortJointInterface</hardwareInterface>
8)      <mechanicalReduction>1</mechanicalReduction>
9)    </actuator>
10)  </transmission>
11)
12)  <transmission name="tran2">
13)    <type>transmission_interface/SimpleTransmission</type>
14)    ***<joint name="joint2">***
15)      ***<hardwareInterface>EffortJointInterface</hardwareInterface>***
16)    ***</joint>***
17)    <actuator name="motor2">
18)      <hardwareInterface>EffortJointInterface</hardwareInterface>
19)      <mechanicalReduction>1</mechanicalReduction>
20)    </actuator>
21)  </transmission>
```

#### Goals of Project ####
There were two goals associated with this project:

1. Create a ROS package that provides a launch file to properly start Gazebo and RViz with the RRBot model loaded. Start a node that sets some PID gains for the joint controllers and creates publishers to have the joints follow sin (i/100).
2. Modify the RRBot definition to add a third link, ie. make it a RRRBot. Leave the camera and laser at the end of the last link. Modify the above node to use the new RRRBot.


#### Project Extensions ####
Possible project extensions include:

* Change the RRRBot to use torque control instead of position control. Write a node that stabalizes the RRRBot to a non-equilibrium configuration.
* Change the RRRBot to use a joint_trajectory_controller. Use this controller to have the RRRBot stabilize to a trajectory.
* Use the joint position controller to have the RRRBot stabilize to a trajectory defined for the end-effector. For example, the end-effector can move vertically up and down while mantaining a constant orientation.
