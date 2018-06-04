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



def pos_callback(data):
	"""
	Position callback, which is executed in the event that a GPS fix is
	published by the Jackal.
	"""
	global global_current_position
	global_current_position = data
	print "jackal_pos_server: jackal's position: {}".format(global_current_position)


def handle_pos_request(req):
	print "Incoming drive request: {}".format(req)
	global global_current_position
	return JackalPosResponse(global_current_position)  # Return current position of Jackal..



def get_jackal_pos_server():
	rospy.init_node('get_jackal_pos_server')
	s = rospy.Service('get_jackal_pos', JackalPos, handle_pos_request)
	rospy.Subscriber('/fix', NavSatFix, pos_callback, queue_size=1)  # Subscribe to Jackal's /fix topic (reach units)
	print "get_jackal_pos_server subscribed to /fix from Jackal.."
	print "Jackal pos server ready.."
	rospy.spin()


if __name__ == "__main__":
	get_jackal_pos_server()