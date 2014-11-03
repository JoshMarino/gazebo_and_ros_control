ME 495: Mini-Project - Gazebo and ROS Control
=============================================

Josh Marino and Mahdieh Javaremi
--------------------------------


## Project Documentation ##


#### Project Overview ####
Determine the relationship between Gazebo, ROS, ROS control, and ROS controllers. This will be implemented using the RRBot model.


#### Useful Tutorials ####
We followed three main tutorials in order to understand how the pieces fit together. These tutorials relied upon the [RRBot github repo](https://github.com/ros-simulation/gazebo_ros_demos.git).

1. [Using a URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf")
2. [Using Gazebo plugins with ROS](http://gazebosim.org/tutorials?tut=ros_gzplugins)
3. [ROS Control](http://gazebosim.org/tutorials/?tut=ros_control)

The [Gazebo website](http://gazebosim.org/) has many more useful tutorials with a variaty of robot simulation models, and a helpful [troubleshooting](https://bitbucket.org/osrf/gazebo/wiki/troubleshooting) link with answers to common problems .

The first tutorial on "Using a URDF in Gazebo" describes the required and optional sections of a URDF. It then goes onto explaining the `rrbot.xacro` file to help build the URDF.

The second tutorial, "using Gazebo plugins with ROS", indroduces several plugins available in gazebo_plugins. These plugis are what enable URDFs to to be used in the Gazebo environement. Two plugins for the camera and hokuyo laser were explained in the `rrbot.gazebo` file.

The third tutorial, "ROS Control", briefly covers the data flow between ros_control and gazebo.  It then delves into using ros_control with a gazebo plug-in to simulate a robot's controllers. A .yaml configuratiom file is first described within a roslaunch file loading the ros parameters. Next, the launch file is roslaunched and the position of the joints are controlled by publishing to their corresponding topic and through using rqt_gui.



#### Tutorial Changes for ROS Indigo ####

The first tutorial begins with some prerequisites before it dives into the URDF usage in Gazebo. If you have already done a full ROS install on your computer, you do not need to install Gazebo. The full install will install a version of Gazebo (in my instance Gazebo 2.0) on your system. Installing Gazebo again will create dependancy conflicts within your catkin workspace.This issue will manifest itself only when you are rebuilding your workspace after downloading, cloning, or creating new packages. 

Another issue that you will face when following the prereq steps is parsing URDF files in Indigo. The answer to this issue can be found [here](http://wiki.ros.org/urdf) under "New in Indigo", section 5.1 Verification. You will need to run 
>sudo apt-get install liburdfdom-tools 

in order to parse URDF files in Indigo. 

In order to successfuly complete the thrid tutorial on ROS Control in Indigo, an adjustment needs to be made in the  `rrbot.xacro` files. The ammendments are shown below in lines 3-5 and 14-16, denoted with ##change##.

```
1)  <transmission name="tran1">
2)    <type>transmission_interface/SimpleTransmission</type>
3)    ##<joint name="joint1">##
4)      ##<hardwareInterface>EffortJointInterface</hardwareInterface>##
5)    ##</joint>##
6)    <actuator name="motor1">
7)      <hardwareInterface>EffortJointInterface</hardwareInterface>
8)      <mechanicalReduction>1</mechanicalReduction>
9)    </actuator>
10)  </transmission>
11)
12)  <transmission name="tran2">
13)    <type>transmission_interface/SimpleTransmission</type>
14)    ##<joint name="joint2">##
15)      ##<hardwareInterface>EffortJointInterface</hardwareInterface>##
16)    ##</joint>##
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

#####Goal 1#####
The three tutorials provide the basic building blocks needed to accomplish goal 1. A complete ROS package that starts gazebo and rviz with the RRBot will require a package.xml, a [.xacro](http://wiki.ros.org/xacro) file used to simplify the robot URDF file and load  , .gazebo and .rviz files to load your robot in the gazebo and rviz simulate and visualise your robot in ROS, .yaml file to incorporate parameters not supported by URDF, an executable .py with the appropriate node definition, and a launch file. 
The .xacro, .gazebo and .rviz files can be made using this [tutorial](http://gazebosim.org/tutorials/?tut=ros_control) as a guide. 

1.Step one: 
First we need to create a configuration file that will contain all parameters that are necessary for our controller. The following is the configuration residing in the .yaml file:

```
#this file represents the controller being used
rrrbot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  
  
  # Position Controllers ---------------------------------------
  joint1_position_controller:
    type: position_controllers/JointPositionController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_position_controller:
    type: position_controllers/JointPositionController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint3_position_controller:
    type: position_controllers/JointPositionController
    joint: joint3
    pid: {p: 100.0, i: 0.01, d: 10.0}
```
The .yaml file is where the controller type is defined. Multiple controllers can be defined in a single .yaml file. When this file contains more than one controller, the ros_control controller_manager can be used to toggle between the different controllers. A sufficient controller definition will require the type of controller(actuation method), the joint it is acting upon, and the gains of the controller (in our case, a common PID controller). Other parameters can be defined in the .yaml. It is of importance to note that when interfacing Gazebo and ros_control, the joint_state_controller paramter must also be defined here. The transmission_interface isn't necissary for simulation in Gazebo, it's importance becomes aparent when you want to control your robot. 

2.Step two: 
Nest we need to create a launch file that will load controller parameters to the parameter server and start up the controller. The launch file will also startup the robot in the world of Gazebo Rviz. 
```
<launch>

  <!-- Load the URDF into the ROS Parameter Server -->
  <param name="robot_description"
    command="$(find xacro)/xacro.py '$(find rrbot_description)/urdf/rrbot.xacro'" />

  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find rrbot_description)/launch/rrbot.rviz"/>

  <!-- Include gazebo launch file rrbot_world.launch -->
  <include file="$(find rrbot_gazebo)/launch/rrbot_world.launch" />

  <!-- Include ros_control launch file to load joint position controllers -->
  <include file="$(find rrbot_control)/launch/rrbot_control.launch" />

  <!-- Create node to control joint positions using joint controller -->
  <node name="joint_positions_node" pkg="rrbot_files" type="position_controller.py" output="screen" respawn="true" />

</launch>
```
The launch file calls the node joint_positions_node, that will publish the desired position message to the Float64 topic.  It also includes the ros_control launch file to load the joint_position_controllers controllers pluggin. 

3.Step three: 
Finally, we need to define a node that will publish the correct message to the Float64 topic which is interpreted by ros_control controller as a desired position. 
```
#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs

#Define a RRBot joint positions publisher for joint controllers.
def rrbot_joint_positions_publisher():

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
		sine_movement = sin(i/100.)

		#Publish the same sine movement to each joint.
		pub1.publish(sine_movement)
		pub2.publish(sine_movement)

		i = i+1 #increment i

		rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: rrbot_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
	
```	

4.Step four: 
In order to control your robot in Gazebo, we also need a control pluggin. We used the basic control pluggin: 

>     <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">


5.Step five: 
The base for interfacing Gazebo and ros_control is the .xacro file. This file will contain all the necessary descriptions to essentially 'build' your robot in the Gazebo world. It accounts for every single joint and link that make up the robot, in which cameras and any attachment is considered a link, and all links are connected via joints. The URDF [links](http://wiki.ros.org/urdf/XML/link) and [joints](http://wiki.ros.org/urdf/XML/joint) must be sufficiently defined in order for Gazebo and Rvis to 'realise' the robot. The transmission_interface and hardware_interface for each joint-actuator pair are also defined here. The Transmission type used in our code is a Simple Reduction Transmission, although depending on the actuator-joint relationship, other transmission methods can be set here. 

#####Goal 2#####
The second goal was to modify the RRBot to an RRRBot, essentially adding an extra link between the previous last link and the camera and laser links. To do this, the .gazebo, .rviz, and .xacro files need to be modified  to account for the new link and joint pair. The .yaml file should be updated to have a controller for the new added joint. The python node needs to publish this new controller command to the new joint. If these changes are made correctly anduniformly, the robot can be extended to as many links as possible with the desired combination of parent and child links. 

#### Project Extensions ####
There are many exciting extensions to this project. ROS_control and gazebo have many capabilities worth exploring. We attempted and succeeded in completing the following extensions: 

* Change the RRRBot to use torque control instead of position control. Write a node that stabalizes the RRRBot to a non-equilibrium configuration.
* Change the RRRBot to use a joint_trajectory_controller. Use this controller to have the RRRBot stabilize to a trajectory.

#####Torque Control#####
In order to include torque control, two main changes need to be made to the existing package. We made this extension ny adding a new folder to our rrrbot_files package, called torque_control. This file contains the new .yaml file which generates torque commands of type JointEffortController instead of JointPositionController. With the new control command, the next addition is to create a publisher node that sends messages to the topic [JointEffortControl](http://wiki.ros.org/robot_mechanism_controllers/JointEffortController). This will work on it's own, however it is better practice to interface with the controller via it's [action interface](http://wiki.ros.org/joint_trajectory_action). 

#####Joint_Trajectory_Controller#####
The joint_trajectory_controller is a bit more tricky to implement than the previous controllers. The trajectory controller takes in a trajectory control command and sends command to a position interface. 
The JointTrajectoryController executes joint-space trajectories on a set of joints.
