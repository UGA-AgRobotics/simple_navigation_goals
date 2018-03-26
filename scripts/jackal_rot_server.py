#!/usr/bin/env python

from simple_navigation_goals.srv import *
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
import PyKDL
from math import degrees, radians, pi



global_current_orientation = None
tf_listener = tf.TransformListener()
odom_frame = '/odom'
base_frame = '/base_link'

cmd_vel = rospy.Publisher('/cmd_vel', Twist)
angular_speed = rospy.get_param("~angular_speed", 0.7)
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians

rate = 20  # 20Hz
# r = rospy.Rate(rate)



def rot_callback_imu(data):
	"""
	Position callback, which is executed in the event that a GPS fix is
	published by the Jackal.
	"""
	global global_current_orientation
	global_current_orientation = data
	angle = quat_to_angle(global_current_orientation.orientation)
	print "Jackal's angle: {}".format(degrees(angle))


def handle_rot_request(req):

	print "Handling request to get Jackal's current orientation.."
	print "Incoming request to be handled: {}".format(req)

	global global_current_orientation

	# Turn Jackal to angle from req:
	# Stop the robot before rotating
	move_cmd = Twist()
	cmd_vel.publish(move_cmd)
	rospy.sleep(1.0)
	
	# Set the movement command to a rotation
	move_cmd.angular.z = angular_speed
	
	# Track the last angle measured
	# curr_angle = quat_to_angle(global_current_orientation.jackal_rot.orientation)
	(position, rotation) = get_odom()
	goal_angle = radians(req.turn_angle)  # get requested angle to turn from client

	print "Current Jackal orientation: {}".format(degrees(rotation))
	print "Goal angle: {}".format(degrees(goal_angle))

	execute_turn(position, rotation, goal_angle, move_cmd)
		
	# Stop the robot when we are done
	cmd_vel.publish(Twist())

	# Return current orientation of Jackal..
	return JackalRotResponse(global_current_orientation)


def execute_turn(position, rotation, goal_angle, move_cmd):
	"""
	Function for executing a turn in the odom frame.
	"""

	turn_angle = 0  # keep track of turning angle
	last_angle = rotation

	# Begin the rotation
	while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
		# Publish the Twist message and sleep 1 cycle         
		cmd_vel.publish(move_cmd)
		
		rospy.sleep(1.0/rate)
		
		# Get the current rotation
		(position, rotation) = get_odom()
		
		
		# Compute the amount of rotation since the last lopp
		delta_angle = normalize_angle(rotation - last_angle)

		print "Turn Rotation: {}".format(degrees(rotation))
		print "Turn delta_angle: {}".format(degrees(delta_angle))
		
		turn_angle += delta_angle
		last_angle = rotation

	return True


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


def get_odom():
	# Get the current transform between the odom and base frames
	try:
		(trans, rot)  = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
		rospy.loginfo("TF Exception")
		return

	return (Point(*trans), quat_to_angle(Quaternion(*rot)))


def get_jackal_rot_server():

	rospy.init_node('get_jackal_rot_server')
	s = rospy.Service('get_jackal_rot', JackalRot, handle_rot_request)

	rospy.Subscriber('/imu/data', Imu, rot_callback_imu)  # Subscribe to Jackal's /navsat/fix topic

	print "get_jackal_rot_server subscribed to /imu/data from Jackal.."
	print "Jackal rot server ready.."

	rospy.spin()



if __name__ == "__main__":
	get_jackal_rot_server()