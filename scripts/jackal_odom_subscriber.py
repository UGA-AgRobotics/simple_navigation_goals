#!/usr/bin/env python

from simple_navigation_goals.srv import *
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
import PyKDL
from math import degrees, radians, pi



global_current_odom = None



def odom_callback(data):
	"""
	Position callback, which is executed in the event that a GPS fix is
	published by the Jackal.
	"""
	global global_current_odom
	global_current_odom = data
	orientation = global_current_odom.pose.pose.orientation
	print "Jackal's angle from /odometry/filtered topic: {}".format(degrees(quat_to_angle(orientation)))


def quat_to_angle(quat):
	"""
	Converts quaternion to angle.
	"""
	rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
	return rot.GetRPY()[2]


def odom_node():
	rospy.init_node('jackal_odom_subscriber', anonymous=True)
	rospy.Subscriber('/odometry/filtered', Odometry, odom_callback, queue_size=1)
	print "Subscribed to /odometry/filtered topic.."
	rospy.spin()



if __name__ == "__main__":
	try:
		odom_node()
	except rospy.ROSInterruptException:
		print "Error occurred running odom_node()!"
		pass