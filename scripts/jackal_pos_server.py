#!/usr/bin/env python

"""
Attempting to create a service that returns the Jackal's current position.

NOTE: Currently, it seems like a service is the way to go with this, but
look more into it to confirm this.

GOAL: Have the navigation node grab the jackal's position that's being 
published on the /navsat/fix topic. So that means either have the jackal's nav
node subscribe to that topic, or do the service thing.

?: Does subscribing to the /navsat/fix topic mean the callback for said topic
will be executed every time a /navsat/fix message is published? If so, the service would
be a good way to go, as we only need to grab the jackal's fix when it's needed.
"""
from simple_navigation_goals.srv import *
import rospy
from sensor_msgs.msg import NavSatFix
import tf
import PyKDL
from math import degrees, radians, pi, sqrt
from geometry_msgs.msg import Twist, Point, Quaternion



global_current_position = None
global_current_orientation = None
tf_listener = tf.TransformListener()
odom_frame = '/odom'
base_frame = '/base_link'
cmd_vel = rospy.Publisher('/cmd_vel', Twist)
linear_speed = 0.3  # units of m/s
# linear_speed = 1.0
rate = 20  # 20Hz



def pos_callback(data):
	"""
	Position callback, which is executed in the event that a GPS fix is
	published by the Jackal.
	"""
	# print "Jackal's lat/lon position: {}, {}".format(data.latitude, data.longitude)
	global global_current_position

	global_current_position = data

	print "jackal_pos_server: jackal's position: {}".format(global_current_position)


def handle_pos_request(req):
	print "Handling request to get Jackal's current position.."
	print "Incoming request to be handled: {}".format(req)

	global global_current_position

	# Return current position of Jackal..
	# return JackalPosResponse(global_current_position)

	# Initialize the movement command
	move_cmd = Twist()
	
	# Set the movement command to forward motion
	move_cmd.linear.x = linear_speed
	
	# Get the starting position values     
	(position, rotation) =  get_odom()
	
	# Keep track of the distance traveled
	goal_distance = req.goal_distance

	print("Driving Jackal {} meters to goal distance..".format(goal_distance))
	
	drive_forward(position, rotation, goal_distance, move_cmd)

	# Stop the robot when we are done
	cmd_vel.publish(Twist())

	# Return current position of Jackal..
	return JackalPosResponse(global_current_position)


def drive_forward(position, rotation, goal_distance, move_cmd):

	distance = 0
	x_start, y_start = position.x, position.y

	# Enter the loop to move along a side
	while distance < goal_distance and not rospy.is_shutdown():
		# Publish the Twist message and sleep 1 cycle         
		cmd_vel.publish(move_cmd)
		
		rospy.sleep(1.0/rate)

		# Get the current position
		(position, rotation) =  get_odom()
		
		# Compute the Euclidean distance from the start
		distance = sqrt(pow((position.x - x_start), 2) + 
						pow((position.y - y_start), 2))

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


def get_jackal_pos_server():
	rospy.init_node('get_jackal_pos_server')
	s = rospy.Service('get_jackal_pos', JackalPos, handle_pos_request)

	# rospy.Subscriber('/navsat/fix', NavSatFix, pos_callback)  # Subscribe to Jackal's /navsat/fix topic
	# print "get_jackal_pos_server subscribed to /navsat/fix from Jackal.."

	rospy.Subscriber('/fix', NavSatFix, pos_callback)  # Subscribe to Jackal's /fix topic (reach units)
	print "get_jackal_pos_server subscribed to /fix from Jackal.."

	print "Jackal pos server ready.."
	rospy.spin()


if __name__ == "__main__":
	get_jackal_pos_server()