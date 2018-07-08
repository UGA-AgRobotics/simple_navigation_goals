#!/usr/bin/env python

from simple_navigation_goals.srv import *
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import PyKDL
from math import degrees



global_current_orientation = None



def rot_callback_imu(rot_msg):
	"""
	Position callback, which is executed in the event that a GPS fix is
	published by the Jackal.
	"""
	global global_current_orientation
	# global_current_orientation = rot_msg.data
	global_current_orientation = rot_msg.orientation

	# angle_in_degrees = degrees(quat_to_angle(global_current_orientation))
	# print("Current angle (degrees): {}".format(angle_in_degrees))



def handle_rot_request(req):

	print("Handling request to get Jackal's current orientation..")

	global global_current_orientation
	# return JackalRotResponse(global_current_orientation)
	return JackalRotResponse(quat_to_angle(global_current_orientation))



def quat_to_angle(quat):
	"""
	Converts quaternion to angle.
	"""
	rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
	return rot.GetRPY()[2]



def get_red_rover_rot_server():

	rospy.init_node('get_red_rover_rot_server')
	s = rospy.Service('get_red_rover_rot', JackalRot, handle_rot_request)
	rospy.Subscriber('/phidget/imu/data', Imu, rot_callback_imu, queue_size=1)
	print("get_red_rover_rot_server subscribed to /imu/data from Red Rover..")
	print("Jackal rot server ready..")
	rospy.spin()



if __name__ == "__main__":
	get_red_rover_rot_server()
