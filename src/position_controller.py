#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


def rrbot_joint_positions_publisher():

	rospy.init_node('joint_positions_node', anonymous=True)

	pub1 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)

	rate = rospy.Rate(100) #100 Hz

	i = 0
	while not rospy.is_shutdown():

		sine_movement = sin(i/100.)

		pub1.publish(sine_movement)
		pub2.publish(sine_movement)

		i = i+1

		rate.sleep()






if __name__ == '__main__':
	try:
		rrbot_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
