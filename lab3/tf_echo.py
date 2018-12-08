#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.


#Import the dependencies as described in example_pub.py
import rospy
import numpy as np
import kin_func_skeleton as kfs
from sensor_msgs.msg import JointState
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped


if __name__ == '__main__':
	rospy.init_node('tf_echo', anonymous=True)
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('left_hand','base',rospy.Time())
			print(trans)
		except(tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
			continue

