#!/usr/bin/env python

"""
This module is for testing the GUI -> Rover nav routine. It'll
read in flags data from a file, then publish to the /sample_points topic
with the flag data sent as a json string.
"""

import roslib
import rospy
import json
import sys
from std_msgs.msg import String
from lib.nav_tracks import NavTracks



sample_points_pub = rospy.Publisher("/sample_points", String, queue_size=10)



def run_sample_points_test(flags):

	rospy.init_node('sample_points_test')

	print("Starting run_sample_points_test node..")

	sample_points_string = json.dumps(flags)  # Is this array valid JSON???

	sample_points_pub.publish(sample_points_string)

	print("run_sample_points_test node ready.")

	rospy.spin()







if __name__ == '__main__':

	try:
		flags_filename = sys.argv[1]
	except IndexError:
		raise Exception("Must provide flags filename..")

	flags_file = open(flags_filename, 'r').read()
	flags = json.loads(flags_file)

	run_sample_points_test(flags)
