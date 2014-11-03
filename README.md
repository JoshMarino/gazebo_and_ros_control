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


#####How do Gazebo, ROS control, and ROS controllers work together?#####


#### Goals of Project ####
There were two goals associated with this project:

1. Create a ROS package that provides a launch file to properly start Gazebo and RViz with the RRBot model loaded. Start a node that sets some PID gains for the joint controllers and creates publishers to have the joints follow sin (i/100).
2. Modify the RRBot definition to add a third link, ie. make it a RRRBot. Leave the camera and laser at the end of the last link. Modify the above node to use the new RRRBot.


#####Goal 1#####
The three tutorials provide the basic building blocks needed to accomplish goal 1. A complete ROS package that starts gazebo and rviz with the RRBot will require:

1. a package.xml with the subscriber/publisher node definition.
2. a [.xacro](http://wiki.ros.org/xacro) file used to simplify the robot URDF file and load. 
3. .gazebo and .rviz files to load your robot in the gazebo and rviz simulate and visualise your robot in ROS.
4. .yaml file to incorporate parameters not supported by URDF, an executable .py with the appropriate node definition, and a launch file. 

The .xacro, .gazebo and .rviz files can be made using this [tutorial](http://gazebosim.org/tutorials/?tut=ros_control) as a guide. 

1. Step one: 
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
This gets loaded to the parameter server in the roslaunch file. The .yaml file is where the controller type is defined. Multiple controllers can be defined in a single .yaml file. When this file contains more than one controller, the ros_control controller_manager can be used to toggle between the different controllers. A sufficient controller definition will require the type of controller(actuation method), the joint it is acting upon, and the gains of the controller (in our case, a common PID controller). Other parameters can be defined in the .yaml. It is of importance to note that when interfacing Gazebo and ros_control, the joint_state_controller paramter must also be defined here. The transmission_interface isn't necissary for simulation in Gazebo, it's importance becomes aparent when you want to control your robot. 

2. Step two: 
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
The launch file calls the node joint_positions_node, which will publish the desired position message to the Float64 topic.  It also includes the ros_control launch file to load the joint_position_controllers controllers pluggin. 

3. Step three: 
Next we need to define a node that will publish the correct message to the Float64 topic which is interpreted by ros_control controller as a desired position. For the purpose of this project, we wanted all our joints to follow a sinusoidal motion $\varphi\colon X\to Y$, see Tag \ref{04FW}
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

4. Step four: 
The base for interfacing Gazebo and ros_control is the .xacro file. This file will contain all the necessary descriptions to essentially 'build' your robot in the Gazebo world. It accounts for every single joint and link that make up the robot, in which cameras and any attachment is considered a link, and all links are connected via joints. The URDF [links](http://wiki.ros.org/urdf/XML/link) and [joints](http://wiki.ros.org/urdf/XML/joint) must be sufficiently defined in order for Gazebo and Rvis to 'realise' the robot. The transmission_interface and hardware_interface for each joint-actuator pair are also defined here. The Transmission type used in our code is a Simple Reduction Transmission, although depending on the actuator-joint relationship, other transmission methods can be set here. 

To simulate and visualize the robot in Gazebo and Rviz, the robot needs to be fully defined. 
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
Every link will need a joint pair for actuation purposes. Every joint connects two links, which are specified as parent (base link) and child link in ROS_control and Gazebo. 

```
  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 ${width} ${height1 - axel_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="2.0"/>
  </joint>
  
```

5. Step five: 
In order to control your robot in Gazebo, several pluggins need to be added to the .gazebo file. For the controller pluggin we used the basic control pluggin: 

>     <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">

End-effectors will commonly have at least one and oftentimes multiple sensors. In our project, we have made use of the camera and hokuyu laser pluggins. 
To use the hokuyo laser scanner, you need to:

1. Define it as a link on the robot. A complete link definition will require the collision, visual and intertial definitions for the link. 

```
<!-- hokuyo -->
  <gazebo reference="hokuyo_link">
    <sensor type="gpu_ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      
```


2. Specify the sampling rate and type of data and noise being measured. 

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
3. Add an additional pluggin to control the hokuyo laser in gazebo. 

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
6. Step 6: 
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
7. Step 7: 
The gazebo.launch file will look for what controller to use for actuating the joints. We create another launch file that specifies the cotroller to be used. This node uses the spawner tool in the  [controller_manager](http://wiki.ros.org/controller_manager) package to load and start the listed controllers.  

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
8. Step eight: 
Now you can launch the simulation of the robot. `roslaunch rrbot_files mini_project.launch position:=true`
The command should load the RRBot in Gazebo and Rviz. 

#####Goal 2#####
The second goal was to modify the RRBot to an RRRBot, essentially adding an extra link between link 3 and the camera and laser links. 

To add this part to our project, we created a sub package called rrrbot_files in the gazebo_and_ros_control package. 
Every new catkin package requires its own package.xml file: 
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
and CMakeLists.txt. 

In our subpackage, we created new .xacro, .gazebo, .yaml, _control.launch and _world.launch files for the RRRBot. 

1. Step 1: 
To create an extra link, the robot definition has to be modified. This will require updating the .gazebo, .rviz, and .xacro files to account for the new link and joint pair.

```
  <!-- Link4 -->
  <gazebo reference="link4">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>Gazebo/Orange</material>
  </gazebo>
```
Note that for each link addition, a joint must also be added.

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
2. Step 2: 
Now that we have an added transmission_interface for the extra joint, we need to define a new controller. Remember that controllers are specified in the .yaml file: 
```
 joint3_position_controller:
    type: position_controllers/JointPositionController
    joint: joint3
    pid: {p: 100.0, i: 0.01, d: 10.0}
```
3. Step 3: 
The new joint3_position_controller is loaded into the parameter server alongside the previous controllers. This controller is added to the controller_manager's list, denoted by ###addition###, to be loaded and started by the spawner tool. 
```
<!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
	output="screen" ns="/rrrbot" args="joint_state_controller
					  joint1_position_controller
					  joint2_position_controller
					  ###joint3_position_controller"/>###
```

4. Step 4: 
The python node needs to publish the control command messages from this added actuator to the new joint. If these changes are made correctly and uniformly, the robot can be extended to as many links as possible with the desired combination of parent and child links, actuation methods and controllers. 

#### Project Extensions ####
There are many exciting extensions to this project. ROS_control and gazebo have many capabilities worth exploring. We attempted and succeeded in completing the following extensions: 

* Change the RRRBot to use torque control instead of position control. Write a node that stabalizes the RRRBot to a non-equilibrium configuration.
* Change the RRRBot to use a joint_trajectory_controller. Use this controller to have the RRRBot stabilize to a trajectory.

#####Torque Control#####
In order to include torque control, two main changes need to be made to the existing package. We made this extension ny adding a new folder to our rrrbot_files package, called torque_control. This file contains the new .yaml file which generates torque commands of type JointEffortController instead of JointPositionController. With the new control command, the next addition is to create a publisher node that sends messages to the topic [JointEffortControl](http://wiki.ros.org/robot_mechanism_controllers/JointEffortController). This will work on it's own, however it is better practice to interface with the controller via it's [action interface](http://wiki.ros.org/joint_trajectory_action). 

#####Mass Balance#####
This extension builds on the torque control of the arm. The goal was to balance the torque on the end effector, when an foreign mass is exterted on the end effecter. The greatest challenge with coding this extension is filtering through all the Gazebo link_state messages to find the onces that are applicable. Two changes were made to the main code. 
1.Step one: 
Add a new block in the launch file to start up the mass balance node. 
2.Step two: 
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
	q = [link4quaternion.x,link4quaternion.y,link4quaternion.z,link4quaternion.w] #creates list from quaternion 												since it was not originally
	link4orientation = euler_from_quaternion(q) #transfrom from quaternion to euler angles
	#Maps the end effector from 3rd revolute joint position and orientation
	end_effector_x = link4pose.position.x + sin(link4orientation[1])
	end_effector_y = link4pose.position.y
	end_effector_z = link4pose.position.z - cos(link4orientation[1])
	end_effector_position = [end_effector_x,end_effector_y,end_effector_z]
	#Proportional controller for torque balance with added mass near the camera
	theta_before_added_object = 21.62 #determined from the supplied torques before added mass
	theta_new = atan2((link4pose.position.z-end_effector_z),(link4pose.position.x-end_effector_x)) #new theta from 													horizion
	Gain = 0.4 #proportional gain
	torque_before_added_object = 9.81 #constant value was applied
	torque_new = (4.1*torque_before_added_object/9.81)*Gain*((theta_new/pi*180) - theta_before_added_object) 											#Torque = (T_before)*K*(delta_theta)
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

that calculates the change in torque required to stabilize the end effector in the presence of an added mass. The new torque is found by augmenting the previous torque by multiplying the change in angle of the end-effector with the controller gain and previous torqe. 

Be sure to import euler_from_quaternion from the tf.transformations, as Gazebo used quaternions to define the c-space of the end-effector. 

>from tf.transformations import euler_from_quaternion


#####Joint_Trajectory_Controller#####
The joint_trajectory_controller is a bit more tricky to implement than the previous controllers. The trajectory controller takes in a trajectory control command and sends command to a position interface. 
The JointTrajectoryController executes joint-space trajectories on a set of joints.
The joint_trajectory_controller works a bit differently than the two previous controllers that were implemented.To send position commands via the joint_position_controller and torque commands via the joint_effort_controller, one need only send messages to a single topic. Contrarily, trajectory commands are sent in the form of the  [trajectory_msgs/JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html) message primarily by means of the action interface, and may also be sent by means of the topic interface. 
