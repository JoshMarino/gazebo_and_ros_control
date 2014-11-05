Gazebo and ROS Control
=============================================

Josh Marino and Mahdieh Nejati
---------------------------------------------

**ME 495:Embedded Systems in Robotics**

**Mini Project, Fall 2014, Northwestern Univeristy**



#### Table of Contents ####
[Project Overview](#Project Overview)

[Useful Tutorials](#Useful Tutorials)

[Tutorial Changes for ROS Indigo](#Tutorial Changes for ROS Indigo)

[Goals of Project](#Goals of Project)

[Goal 1](#Goal 1)

[Goal 2](#Goal 2)

[Project Extensions](#Project Extensions)

[Torque Control](#Torque Control)

[Mass Balance](#Mass Balance)

[Joint_Trajectory_Controller](#Joint_Trajectory_Controller)



#### Project Overview  <a name="Project Overview"></a>
ROS_control is an exciting new development in the world of ROS, created and maintained by Adolfo Rodriquez Tsouroukdissian, and maintained by him <adolfo.rodriguez@pal-robotics.com>, Dave Coleman <davetcoleman@gmail.com>, and Wim Meeussen <wim@hidof.com>. As with any ROS package, the documentaion on different aspects of this multi-faceted package are lacking and dependent on the ROS communities' implementation and subsequent documentation. The purpose of this project was to understand how ros_control and the world of Gazebo can communicate together in order to simulate robot actuation and sensing mechanisms. Specifically, we wanted to understand:

1. How ROS controllers and ROS control work.
2. Using the RRBot model, how do we use ROS controllers to simulate a robot in Gazebo.
3. What are the different ways to simulate robotic controllers in Gazebo. 


#### Useful Tutorials  <a name="Useful Tutorials"></a>
We followed three main tutorials in order to understand how the pieces fit together. These tutorials relied upon the [RRBot github repo](https://github.com/ros-simulation/gazebo_ros_demos.git).

1. [Using a URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf")
2. [Using Gazebo plugins with ROS](http://gazebosim.org/tutorials?tut=ros_gzplugins)
3. [ROS Control](http://gazebosim.org/tutorials/?tut=ros_control)

The [Gazebo website](http://gazebosim.org/) has many more useful tutorials with a variety of robot simulation models, and a helpful [troubleshooting](https://bitbucket.org/osrf/gazebo/wiki/troubleshooting) link with answers to common problems.

The first tutorial on "Using a URDF in Gazebo" describes the required and optional sections of a URDF. It then goes onto explaining the `rrbot.xacro` file to help build the URDF.

The second tutorial, "using Gazebo plugins with ROS", indroduces several plugins available in gazebo_plugins. These plugins are what enable URDFs to to be used in the Gazebo environment. Two plugins for the camera and hokuyo laser were explained in the `rrbot.gazebo` file, and used in our project. 

The third tutorial, "ROS Control", briefly covers the data flow between ros_control and Gazebo.  It then delves into using ros_control with a gazebo plug-in to simulate a robot's controllers. A .yaml configuratiom file is first described within a roslaunch file for loading the ROS parameters. Next, the launch file is roslaunched and the position of the joints are controlled by publishing to their corresponding topic and through using rqt_gui.


#### Tutorial Changes for ROS Indigo  <a name="Tutorial Changes for ROS Indigo"></a>
* The first tutorial begins with some prerequisites before it dives into the URDF usage in Gazebo. If you have already done a full ROS install on your computer, you do not need to install Gazebo. The full install will install a version of Gazebo on your system. Installing Gazebo again will create dependancy conflicts within your catkin workspace. This issue will manifest itself only when you are rebuilding your workspace after downloading, cloning, or creating new packages. 

* Another issue that you will face when following the prerequisite steps is parsing URDF files in Indigo. The answer to this issue can be found [here](http://wiki.ros.org/urdf) under "New in Indigo" in section 5.1 "Verification". You will need to run 

  > sudo apt-get install liburdfdom-tools 

 in order to parse URDF files in Indigo. 

* In order to successfuly complete the thrid tutorial on ROS Control in Indigo, an adjustment needs to be made in the `rrbot.xacro` file. The ammendments are shown below in lines 3-5 and 14-16, denoted with ##change##.

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


**How do Gazebo, ROS control, and ROS controllers work together?**


#### Goals of Project  <a name="Goals of Project"></a>
There were two goals associated with this project:

1. Create a ROS package that provides a launch file to properly start Gazebo and RViz with the RRBot model loaded. Start a node that sets some PID gains for the joint controllers and creates publishers to have the joints follow sin(i/100). 
2. Modify the RRBot definition to add a third link, ie. make it a RRRBot. Leave the camera and laser at the end of the last link. Modify the above node to use the new RRRBot.

**ROS package Dependancies**
* Gazebo: The main ROS simulation environment. 
* RViz: 3D visualisation tool for ROS. 
* ROS Indigo: Eighth ROS distribution release. 
* [ros_control](https://github.com/ros-controls/ros_control): Controller framework for ROS
* ros_controllers `sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controller`

#####Goal 1 <a name="Goal 1"></a>
The three tutorials provide the basic building blocks needed to accomplish the first goal. A complete ROS package that starts Gazebo and RViz with the RRBot will require:

1. a package.xml and CMakeList file to build within the CatKin workskpace
2. a [.xacro](http://wiki.ros.org/xacro) file used to simplify the robot URDF file and load
3. .gazebo and .rviz files to load your robot in Gazebo and Rviz for robot simulation and visualisation in ROS
4. .yaml file to incorporate parameters not supported by URDF
5. an executable script with the appropriate subscriber/publisher node definition
6. a launch file

The .xacro, .gazebo and .rviz files can be made using this [tutorial](http://gazebosim.org/tutorials/?tut=ros_control) as a guide. 

*1) Step one:*
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
This gets loaded to the parameter server in the roslaunch file. The .yaml file is where the controller type is defined. Multiple controllers can be defined in a single .yaml file. When this file contains more than one controller, the ros_control [controller_manager](http://wiki.ros.org/controller_manager) in the parameter server can be used to toggle between the different controllers. 
A sufficient controller definition will require the type of controller (actuation method), the joint it is acting upon, and the gains of the controller. For this project we used a PID controller since we could use the tutorial to tune the gains nicely for the RRBot. Other parameters can be defined in the .yaml. It is of importance to note that when interfacing Gazebo and ros_control, the joint_state_controller paramter must also be defined here. The transmission_interface isn't necessary for simulation in Gazebo, it's importance becomes apparent when you want to control your robot. 

*2) Step two:*
Next we need to create a launch file that will load controller parameters to the parameter server and start up the controller. The launch file will also start up the robot in the world of Gazebo and RViz. 

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
The launch file calls the node joint_positions_node, which will publish the desired position message to the Float64 topic.  It also includes the ros_control launch file to load the joint_position_controllers controller plugin. 

*3) Step three:*
Next we need to define a node that will publish the message of type [Float64](http://docs.ros.org/indigo/api/std_msgs/html/msg/Float64.html) which is interpreted by the ROS controller as the desired joint position. For the purpose of this project, we wanted our joints to follow a sinusoidal motion defined by sin(i/100).

```
#!/usr/bin/env python

import rospy
import mathn

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

*4) Step four:*
The basis for interfacing Gazebo and ros_control is the .xacro file. This file will contain all the necessary descriptions to essentially 'build' your robot in the Gazebo world. It accounts for every joint and link that make up the robot, in which cameras and any attachment is considered a link, and all links are connected via joints. The URDF [links](http://wiki.ros.org/urdf/XML/link) and [joints](http://wiki.ros.org/urdf/XML/joint) must be sufficiently defined in order for Gazebo and RViz to define the robot.

The transmission_interface and hardware_interface for each joint-actuator pair are also defined here. The transmission type used in our code is a Simple Reduction Transmission, although depending on the actuator-joint relationship, other transmission methods can be set here. 

To simulate and visualize the robot in Gazebo and RViz, the links need to have three well-defined tags:
 * collision
 * visual
 * interial
```
  <link name="link1">
    <collision>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height1}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height1}"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <inertial>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
	  ixx="1.0" ixy="0.0" ixz="0.0"
	  iyy="1.0" iyz="0.0"
	  izz="1.0"/>
    </inertial>
  </link>
```
Every link will need a joint pair for actuation purposes. Every joint connects two links, which are specified as parent (base link) and child link in ros_control and Gazebo. 

```
  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 ${width} ${height1 - axel_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="2.0"/>
  </joint>
  
```

*5) Step five:*
In order to control your robot in Gazebo, several plugins need to be added to the .gazebo file. For the controller plugin we used the basic control plugin: 

>     <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">

End-effectors will typically have at least one, and oftentimes, multiple sensors. In our project, we have made use of the camera and hokuyu laser plugins. 

To use the hokuyo laser scanner, you need to:

  a. Define it as a link on the robot. A complete link definition will require the collision, visual and intertial definitions for the link. 

```
<!-- hokuyo -->
  <gazebo reference="hokuyo_link">
    <sensor type="gpu_ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
```

  b. Specify the sampling rate, type of data, and noise being measured. 

```
     <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
               reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
```
  c. Add an additional plugin to control the hokuyo laser in gazebo. 

```
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_gpu_laser.so">
        <topicName>/rrrbot/laser/scan</topicName>
        <frameName>hokuyo_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

```

The same steps apply to the camera sensor. 
```
  <!-- camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.  
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>rrrbot/camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo> 
```
*6) Step 6:*
Now we need to create a launch file that will load the robot into the gazebo world. This file will send a service call to gazebo_ros to spawn the RRBot in the empty gazebo world. 

```
<launch>

  <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find rrbot_gazebo)/worlds/rrbot.world"/>
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>

  <!-- Load the URDF into the ROS Parameter Server -->
  <param name="robot_description"
	 command="$(find xacro)/xacro.py '$(find rrrbot_files)/trajectory_control/rrrbot_trajectory.xacro'" />

  <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
	args="-urdf -model rrrbot -param robot_description"/>

  <!-- ros_control rrbot launch file -->
  <!--include file="$(find rrbot_control)/launch/rrbot_control.launch" /-->

</launch>

```
*7) Step 7:*
The gazebo.launch file will look for what controller to use for actuating the joints. We create another launch file that specifies the controller to be used. This node uses the spawner tool in the  [controller_manager](http://wiki.ros.org/controller_manager) package to load and start the listed controllers.

```
<launch>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find rrbot_files)/rrbot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
	output="screen" ns="/rrrbot" args="joint_state_controller
					  joint1_position_controller
					  joint2_position_controller"/>
					 
  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
	respawn="false" output="screen">
    <remap from="/joint_states" to="/rrrbot/joint_states" />
  </node>

</launch>
```

*8) Step eight:*
Now you can launch the ros_control robot simulation in Gazebo: `roslaunch rrbot_files mini_project.launch position:=true`.
The command should load the RRBot in Gazebo and RViz. 

![RRBOT Image](default_gzclient_camera(0)-2014-11-02T13_11_07.416660.jpg)

#####Goal 2 <a name="Goal 2"></a>
The second goal was to modify the RRBot to an RRRBot, essentially adding an extra link between link 3 and the camera and laser links. 

To add this part to our project, we created a sub package called rrrbot_files in the gazebo_and_ros_control package. 
Every new catkin package requires its own package.xml file and CMakeLists.txt: 
```
<?xml version="1.0"?>
<package>
	<name>rrrbot_files</name>
	<version>0.0.0</version>
	<description>The mini_project package</description>

	<maintainer email="josh@todo.todo">josh</maintainer>

	<license>TODO</license>

	<buildtool_depend>catkin</buildtool_depend>
	<build_depend>roscpp</build_depend>
	<build_depend>rospy</build_depend>
	<build_depend>std_msgs</build_depend>
	<run_depend>roscpp</run_depend>
	<run_depend>rospy</run_depend>
	<run_depend>std_msgs</run_depend>

</package>
```


Additionally, for our new sub-package we modified our previous files to work with the RRRBot definition: 
	* .xacro
	* .gazebo
	* .yaml
	* _control.launch 
	* _world.launch 
	
Essentially RRBot was modified to RRRBot, and the package name and file path were amended.

The one thing that does not require any change is the name of the world to be launched. This part in the _world.launch file remains the same: ' <arg name="world_name" value="$(find rrbot_gazebo)/worlds/rrbot.world"/>'.

*1) Step one:*
To create an extra link, the robot definitions have to be modified in the package files. The .gazebo file is updated with the new link definition: 
```
  <!-- Link4 -->
  <gazebo reference="link4">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>Gazebo/Orange</material>
  </gazebo>
```
The .xacro file accounts for link-joint pairs, and the actuators. So for each link addition, a joint must also be added: 

New link definition:
```
  <link name="link4">
    <collision>
      <origin xyz="0 0 ${height4/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height4}"/>
      </geometry>
    </collision>
```
New joint definition: 
```
  <joint name="joint3" type="continuous">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 ${width} ${height3 - axel_offset*2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="2.0"/>
  </joint>
```
The parent link for the camera and hokuyu laser joints must be modified. The modification is denoted with ###change###.

Hokuyu laser scanner:
```
  <joint name="hokuyo_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="0 0 ${height3 - axel_offset/2}" rpy="0 0 0"/>
   ### <parent link="link4"/>###
    <child link="hokuyo_link"/>
  </joint>
```
Camera: 
```
  <joint name="camera_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="${camera_link} 0 ${height4 - axel_offset*2}" rpy="0 0 0"/>
    ###<parent link="link4"/>###
    <child link="camera_link"/>
  </joint>
```
A new transmission needs to be added to account for the actuation of the added joint3: 
```
  <transmission name="tran3">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint3">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor3">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
   ```
*2) Step two:* 
Now that we have an added transmission_interface for the extra joint, we need to define a new controller. Remember that controllers are defined in the .yaml file: 
```
 joint3_position_controller:
    type: position_controllers/JointPositionController
    joint: joint3
    pid: {p: 100.0, i: 0.01, d: 10.0}
```
*3. Step three:* 
The new joint3_position_controller is loaded into the parameter server alongside the previous controllers. This controller is added to the controller_manager's list, denoted by ###addition###, to be loaded and started by the spawner tool. 
```
<!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
	output="screen" ns="/rrrbot" args="joint_state_controller
					  joint1_position_controller
					  joint2_position_controller
					  ###joint3_position_controller"/>###
```
*4) Step four:*
The python node needs to publish the control command messages from the added actuator to the new joint.

> pub3 = rospy.Publisher('/rrrbot/joint3_position_controller/command', Float64, queue_size=10)

> pub3.publish(sine_movement)

*5) Step 5:*
Now you can launch the simulation of the robot in Gazebo: `roslaunch rrrbot_files rrrbot_launch.launch position:=true`
The command should load the RRRBot in Gazebo and RViz. 

If these changes are made correctly and uniformly, the robot can be extended to as many links as possible with the desired combination of parent and child links, actuation methods and controllers. 

![RRRBOT Image](default_gzclient_camera(0)-2014-11-02T13_13_39.164758.jpg?raw=tru)

#### Project Extensions  <a name="Project Extensions"></a>
There are many exciting extensions to this project. ros_control and Gazebo have many capabilities worth exploring, and their integration has not been fully documented yet. We attempted and succeeded in completing the following extensions: 

* Change the RRRBot to use torque control instead of position control. Write a node that stabalizes the RRRBot to a non-equilibrium configuration.
* Balance the end-effector to a non-equilibrium configuration in the presence of an added mass. 

The following extension was investigated and attempted. However, we were not able to implement joint_trajectory_controller in Gazebo:

* Change the RRRBot to use a joint_trajectory_controller. Use this controller to have the RRRBot stabilize to a trajectory.

#####Torque Control  <a name="Torque Control"></a>
Torque control in ros_control and Gazebo is done using the Joints_Effort_Controller, where effort can mean force or torque. We found that implementing torque control in Gazebo is not conceptually different from implementing position control. The main difference is that with position control, in the absence of joint limits, the robot can be commanded to any position in c-space. However, with torque control, once you command enough torque for the joint-link to cross the horizon, you give it enough energy for it to fall backwards under the force of gravity and become animated.

To implement torque control, two main changes needed to be made to the existing package. We made this extension by adding a new folder to our rrrbot_files package, called torque_control. 

*1) Step one:*
We created a .yaml file which generates torque commands of type JointEffortController instead of JointPositionController. Notice that we still require the joint_state_controller to publish the current state of each joint. 
```
rrrbot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  


  # Effort Controllers ---------------------------------------
  joint1_torque_controller:
    type: effort_controllers/JointEffortController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_torque_controller:
    type: effort_controllers/JointEffortController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint3_torque_controller:
    type: effort_controllers/JointEffortController
    joint: joint3
    pid: {p: 100.0, i: 0.01, d: 10.0}

```
*2) Step two:*
With the new control command, the next addition is to create a publisher node that sends messages to the topic [JointEffortControl](http://wiki.ros.org/robot_mechanism_controllers/JointEffortController). To interface this topic with Gazebo, you can use the topic on its own. However, to control an actaul robot it is better practice to interface with the controller via it's [action interface](http://wiki.ros.org/joint_trajectory_action). 

*3) Step three:*
Next we modify the rrrbot_control_launch.launch file, where the control manager resides. This launcher will load the torque controller parameters into the parameter server, and the spawner tool will load and unload the effort controllers listed in the controller_manager. 
```
<launch>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find rrrbot_files)/torque_control/rrrbot_torque_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
	output="screen" ns="/rrrbot" args="joint_state_controller
					  joint1_torque_controller
					  joint2_torque_controller
					  joint3_torque_controller"/>

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
	respawn="false" output="screen">
    <remap from="/joint_states" to="/rrrbot/joint_states" />
  </node>

</launch>
```
*4) Step four:*
Next we need to edit our launch file to start up the node that will publish effort commands. 

``` 
<!-- Run joint torque controllers if true -->
  <group if="$(arg torque)">
    <!-- Include ros_control launch file to load joint torque controllers -->
    <include file="$(find rrrbot_files)/torque_control/rrrbot_torque_control.launch" />

    <!-- Create node to control joint torques using effort controller -->
    <node name="rrrbot_joint_torques_node" pkg="rrrbot_files" type="rrrbot_torques_controller.py" output="screen" respawn="true" />
  </group>

```
Now that we have two different actuation messages, to be able to control which controllers command the robot joints from the launch command, we add the roslaunch [arg](http://wiki.ros.org/roslaunch/XML/arg) tag. 

```
  <arg name="position" default="false"/>  
  <arg name="torque" default="false"/>

```
*5) Step five:*
Finally, you need to create the node that will publish the joint_effort_controller messages to the appropriate robot joints. 
```

#Define a RRRBot joint positions publisher for joint controllers.
def rrrbot_joint_torques_publisher():


	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrrbot_joint_torque_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/rrrbot/joint1_torque_controller/command', Float64, queue_size=100)
	pub2 = rospy.Publisher('/rrrbot/joint2_torque_controller/command', Float64, queue_size=100)
	pub3 = rospy.Publisher('/rrrbot/joint3_torque_controller/command', Float64, queue_size=100)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
		torque = 9.81


		#Publish the same sine movement to each joint.
		if i < 100:
			pub1.publish(2.5*torque)
			pub2.publish(1.2*torque)
			pub3.publish(4.1*torque/9.81)
		else:
			gazebo_link_states()

		i = i+1

```
The top three links in our robot have the following inertial definition in the .xacro file: 
```
    <inertial>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
	  ixx="1.0" ixy="0.0" ixz="0.0"
	  iyy="1.0" iyz="0.0"
	  izz="1.0"/>
    </inertial>
```
The links have moments of intertia of 1 about each axis, a mass of 1kg, and the center of mass located in the center of each link. What this essentially means is that we have a point mass at a distance of 1/2(link length) away from where the effort is being applied. This emplies the maximum torque we can apply to the end-effector link before it cross the horizon is: 

torque = Force(Length/2)

torque = mass*gravitational acceleration*(Length of link)/2

torque = 1*9.81*1/2*1=4.905 Nm

We found that a nice point to stabilize the arm to would be:

* Torque: 9.81 Nm
* Effort on Link 1 (from base): 2.5xtorque (Nm)
* Effort on Link 2: 1.2xtorque (Nm)
* Effort on Link 3: 4.1 (Nm)

*6) Step six:*
Now we can roslaunch the joint_effort_controller simulation in Gazebo: `roslaunch rrrbot_files rrrbot_launch.launch torque:=true`
The command should load the RRRBot in Gazebo and RViz. 

#####Mass Balance  <a name="Mass Balance"></a>
This extension builds on the torque control of the arm. The goal was to balance the torque on the end effector, when a foreign mass is placed on the end effector. The greatest challenge with coding this extension is filtering through all the Gazebo link_state messages to find the ones that are applicable. Two changes were made to the main code. 

*1) Step one:*
Add a new block in the launch file to start up the mass balance node. 
```

  <!-- Run joint torque controllers for added mass at end if true -->
  <group if="$(arg torque_extension)">
    <!-- Include ros_control launch file to load joint torque controllers -->
    <include file="$(find rrrbot_files)/torque_control/rrrbot_torque_control.launch" />

    <!-- Create node to control joint torques using effort controller -->
    <node name="rrrbot_joint_torques_node" pkg="rrrbot_files" type="rrrbot_torques_controller_extension.py" output="screen" respawn="true" />
  </group>
```

We also added another roslaunch arg tag: 

>   <arg name="torque_extension" default="false"/>

*2) Step two:* 
Create a new node that subscribes to the Gazebo [link_states](http://docs.ros.org/indigo/api/gazebo_msgs/html/msg/LinkStates.html) messages. We created a new executable file rrrbot_joint_torques_controller_extenstion.py. This file is built upon our joint_torques_control.py file from the previous extension. 

```
#Initiates Subscriber to gazebo/link_states
def gazebo_link_states():
rospy.Subscriber("/gazebo/link_states", LinkStates, callback) #subscribes to /gazebo/link_states of type LinkStates
rospy.spin()
```
Be sure to import the topic into your exectuble node: `from gazebo_msgs.msg import LinkStates`. 

Next, you must use the messages from the link_states topic to send a joint_efforts_controller command that will compensate for the added mass on the end-effector. 

```
#Callback from Subscriber for Torque Balance with Added Mass
def callback(message):
#Obtains 3rd revolute joint position and orientation from gazebo/link_states
	link4pose = message.pose[4]
	link4quaternion = link4pose.orientation
	q = [link4quaternion.x,link4quaternion.y,link4quaternion.z,link4quaternion.w] #creates list from quaternion since it was not originally
	link4orientation = euler_from_quaternion(q) #transfrom from quaternion to euler angles

	#Maps the end effector from 3rd revolute joint position and orientation
	end_effector_x = link4pose.position.x + sin(link4orientation[1])
	end_effector_y = link4pose.position.y
	end_effector_z = link4pose.position.z - cos(link4orientation[1])
	end_effector_position = [end_effector_x,end_effector_y,end_effector_z]

	#Proportional controller for torque balance with added mass near the camera
	theta_before_added_object = 21.62 #determined from the supplied torques before added mass
	theta_new = atan2((link4pose.position.z-end_effector_z),(link4pose.position.x-end_effector_x)) #new theta from horizion

	Gain = 0.4 #proportional gain

	torque_before_added_object = 9.81 #constant value was applied
	torque_new = (4.1*torque_before_added_object/9.81)*Gain*((theta_new/pi*180) - theta_before_added_object) #Torque = (T_before)*K*(delta_theta)

	#Re-initializing publishers from before since in a different function
	pub1 = rospy.Publisher('/rrrbot/joint1_torque_controller/command', Float64, queue_size=100)
	pub2 = rospy.Publisher('/rrrbot/joint2_torque_controller/command', Float64, queue_size=100)
	pub3 = rospy.Publisher('/rrrbot/joint3_torque_controller/command', Float64, queue_size=100)

	#Publishing new torques to topics
	pub1.publish(2.5*torque_before_added_object) #no difference
	pub2.publish(1.2*torque_before_added_object) #no difference
	pub3.publish((4.1*torque_before_added_object/9.81)+torque_new) #torque_before + torque_new

	#Debugging info - difference in theta & torque_new
	print "Theta Diff: ", (theta_new/pi*180-theta_before_added_object), "\t", "New Torque: ", torque_new
```

This calculates the change in torque required to stabilize the end effector in the presence of an added mass. The new torque is found by augmenting the previous torque by multiplying the change in angle of the end-effector with the controller gain and previous torque. 

Be sure to import euler_from_quaternion from the tf.transformations, as Gazebo used quaternions to define the c-space of the end-effector. 

>from tf.transformations import euler_from_quaternion

![mass balance](default_gzclient_camera(0)-2014-11-04T14_34_26.631106.jpg)

#####Joint_Trajectory_Controller  <a name="Joint_Trajectory_Controller"></a>
The joint_trajectory_controller is a bit more tricky to implement than the previous controllers.The JointTrajectoryController executes joint-space trajectories on a set of joints. It takes in a trajectory control command and sends command to a position interface. There are a few tutorials online that talk about implementing joint_trajectory_control on actual robots. However, we were not able to find any tutorials that interfaced joint_trajectory_control in Gazebo. From our investigation were able to deduce information about aspects of ros_control differs from effort and position control.  

The joint_trajectory_controller works a bit differently than the two previous controllers that were implemented. To send position commands via the joint_position_controller and torque commands via the joint_effort_controller, one need only send messages to a single topic. Contrarily, trajectory commands are sent in the form of the [trajectory_msgs/JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html) message primarily by means of the action interface, and may also be sent by means of the topic interface. 
