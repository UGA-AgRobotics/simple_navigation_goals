#!/usr/bin/env python

"""
Potential node for handling the robot's position relative to flags of interest.

This node could read in the a flags file, and check the robot's distance to
the goals. When the robot is within a given distance window from a goal, stop the
robot and send a signal (a light for now, but a signal to the robot arm later) to
indicate to the user that the Jackal is at the goal.

Subscribes: /fix - the emlid reach gps topic (todo: change topic name to /reach/fix)
Publishes: /at_flag - boolean type, main drive routine will subscribe to this and stop jackal if True
"""

import sys
import math
import rospy
import utm
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import flag_file_handler  # local requirement
import nav_tracks  # local requirement



flag_publisher = rospy.Publisher('at_flag', Bool, queue_size=10)
flags_global = None  # where flags in format of list of utm pairs is stored (todo: make into class?)
flag_tolerance = 0.8  # distance to flag to consider being at said flag (units: meters)



def get_utm_from_fix(current_fix):
	"""
	Converts fix object with 'latitude' and 'longitude' to utm
	"""
	return utm.from_latlon(current_fix.latitude, current_fix.longitude)



def compare_position_to_flags(current_utm):
	"""
	Loops through flags and calculates distance from current
	position to the flags. If within some distance, publish on /at_flag
	topic to tell robot to stop!
	"""
	global flags_global
	global flag_tolerance

	print("Flag tolerance: {}".format(flag_tolerance))

	flag_counter = 1
	for flag_pos in flags_global:
		distance = math.sqrt((current_utm[1] - flag_pos[1])**2 + (current_utm[0] - flag_pos[0])**2)
		print("Flag {} distance: {}".format(flag_counter, distance))

		if distance <= flag_tolerance:
			print("robot has reached the flag within given tolerance!")
			flag_publisher.publish(True)  # Publishes to drive routine to stop robot at the flag
			break

		flag_counter += 1

	return



def position_callback(current_fix):
	"""
	Position callback, which is executed in the event that a GPS fix is
	published by the Jackal.
	"""
	# print "jackal_pos_server: jackal's position: {}".format(current_fix)
	current_utm = get_utm_from_fix(current_fix)  # converts current fix to utm
	compare_position_to_flags(current_utm)




def start_flag_node(flags):

	# Sets global flags variable to be used by module
	global flags_global
	flags_global = flags

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('flag_node', anonymous=True)

	rospy.Subscriber("/fix", NavSatFix, position_callback, queue_size=1)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()








if __name__ == '__main__':

	_flag_filename = None  # input arg for filename of flags file

	try:
		_flag_filename = sys.argv[1]
	except IndexError:
		raise Exception("Must provide a flag filename when running this node..")

	# Reads in flags file:
	fh = flag_file_handler.FlagFileHandler()  # instantiates flag handler
	fh.read_flags_file(_flag_filename)  # reads in flags file
	fh.fill_out_flags_file()  # fills out any missing formats for flag data (dsm, dec, utm)

	# Uses nav_tracks module to convert flags file to list of [easting, northing] pairs:
	nt = nav_tracks.NavTracks()  # instantiates nav_tracks module
	flags = nt.get_track_from_course(fh.flags)  # converts flags file to list of utm pairs

	try:
		start_flag_node(flags)
	except rospy.ROSInterruptException:
		pass