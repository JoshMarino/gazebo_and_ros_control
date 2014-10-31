#!/usr/bin/env python

import rospy
import math
import time

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


#Define a RRBot joint positions publisher for joint controllers.
def rrrbot_joint_torques_publisher():


	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrrbot_joint_torque_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/rrrbot/joint1_torque_controller/command', Float64, queue_size=100)
	pub2 = rospy.Publisher('/rrrbot/joint2_torque_controller/command', Float64, queue_size=100)
	pub3 = rospy.Publisher('/rrrbot/joint3_torque_controller/command', Float64, queue_size=100)

	pub1.publish(0)
	pub2.publish(0)
	pub3.publish(0)
	time.sleep(10)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
		torque = 9.81/3

		#Publish the same sine movement to each joint.
		pub1.publish(torque)
		pub2.publish(torque)
		pub3.publish(torque)

		#i = i+1 #increment i

		rate.sleep() #sleep for rest of rospy.Rate(100)





#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: rrrbot_joint_torques_publisher()
	except rospy.ROSInterruptException: pass
