#!/usr/bin/env python
import rospy
import tf.transformations as tr
import tf
import math
import numpy as np

i = 0
thetha_x = 0
thetha_y = 0
rospy.init_node('tf_publisher_node')
br = tf.TransformBroadcaster()
while not rospy.is_shutdown():
	thetha_x = thetha_x + np.pi/180
	thetha_y = thetha_y + np.pi/180
	rate = rospy.Rate(10.0)
	x = 2*math.cos(thetha_x)
	y = 2*math.sin(thetha_y)
	br.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(), 'base_link',"world") #####the transformation is published here
	rate.sleep()


