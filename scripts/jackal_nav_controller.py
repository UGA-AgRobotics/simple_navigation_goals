#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from simple_navigation_goals.srv import *
import tf
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL



# Global settings:
tf_listener = tf.TransformListener()
odom_frame = '/odom'
base_frame = '/base_link'
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
linear_speed = 0.4  # units of m/s
rate = 20  # Hz
angular_speed = 0.4
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians




def drive_forward(goal_distance, look_ahead):

	print("Driving Jackal {} meters..".format(goal_distance))

	move_cmd = Twist()
	move_cmd.linear.x = linear_speed

	(position, rotation) =  get_odom()  # Get the starting position values

	distance = 0
	x_start, y_start = position.x, position.y

	# Enter the loop to move along a side
	while distance < (goal_distance - look_ahead) and not rospy.is_shutdown():
		# Publishes the Twist message and sleep 1 cycle         
		cmd_vel.publish(move_cmd)
		rospy.sleep(1.0/rate)

		(position, rotation) =  get_odom()  # Get the current position
		
		# Compute the Euclidean distance from the start
		distance = sqrt(pow((position.x - x_start), 2) + 
						pow((position.y - y_start), 2))

	return True



def execute_turn(goal_angle):
	"""
	Function for executing a turn in the odom frame.
	"""
	move_cmd = Twist()

	if goal_angle > 0:
		move_cmd.angular.z = angular_speed

	if goal_angle < 0:
		move_cmd.angular.z = -angular_speed
	
	(position, rotation) = get_odom()

	turn_angle = 0  # keep track of turning angle
	last_angle = rotation

	# Begin the rotation
	while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
		# Publishes the Twist message and sleep 1 cycle:         
		cmd_vel.publish(move_cmd)
		rospy.sleep(1.0/rate)
		(position, rotation) = get_odom()  # Get the current rotation		
		delta_angle = normalize_angle(rotation - last_angle)  # Compute the amount of rotation since the last lopp
		turn_angle += delta_angle
		last_angle = rotation

		if delta_angle == 0.0:
			print "Turned {} degrees..".format(degrees(last_angle))
			break

	return True



def get_odom():
	# Get the current transform between the odom and base frames
	try:
		(trans, rot)  = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
		rospy.loginfo("TF Exception")
		return

	return (Point(*trans), quat_to_angle(Quaternion(*rot)))



def quat_to_angle(quat):
	"""
	Converts quaternion to angle.
	"""
	rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
	return rot.GetRPY()[2]



def normalize_angle(angle):
	res = angle
	while res > pi:
		res -= 2.0 * pi
	while res < -pi:
		res += 2.0 * pi
	return res