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
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
import json
# import ..flag_file_handler  # local requirement
# import ..nav_tracks  # local requirement
from simple_navigation_goals.scripts import flag_file_handler
from simple_navigation_goals.scripts import nav_tracks



class FlagHandler:

	def __init__(self, flags=None):

		print("Initializing jackal_flags_node..")
		
		rospy.init_node('flag_node', anonymous=True)

		# Subscribers:
		rospy.Subscriber("/fix", NavSatFix, self.position_callback, queue_size=1)
		rospy.Subscriber('/sample_points', String, self.sample_points_callback, queue_size=10)  # indicates to rover sample is collected, drive to next flag

		# Publishers:
		self.flag_publisher = rospy.Publisher('/at_flag', Bool, queue_size=1)
		
		self.flag_tolerance = 0.5  # distance to flag to consider being at said flag (units: meters)
		self.flag_index = 0  # Index of the robot's current flag it's going toward
		self.flag_run_complete = False
		self.flags = flags  # where flags in format of list of utm pairs is stored

		# if not self.flags:
		# 	raise Exception("Must project jackal_flags_node with flags file")

		print("Flags list: {}".format(self.flags))
		print("Flag tolerance: {}".format(self.flag_tolerance))

		print("jackal_flags_node ready.")

		rospy.spin()



	def sample_points_callback(self, msg):
		"""
		Reads in flags data (geojson format) and starts the
		red rover's navigation to the flags (in order).
		"""
		print("Received message from /sample_points topic. Loading received flags..")
		flags_obj = json.loads(msg.data)

		# Convert flags into list of [easting, northing] pairs..
		nt = nav_tracks.NavTracks()
		flags_array = nt.get_flags_from_geojson(flags_obj)  # returns list of [easting, northing] pairs
		self.flags = flags_array

		print("Flags: {}".format(flags_array))

		print("Publishing to Red Rover's drive node to initiate driving..")


		


	def get_utm_from_fix(self, current_fix):
		"""
		Converts fix object with 'latitude' and 'longitude' to utm
		"""
		return utm.from_latlon(current_fix.latitude, current_fix.longitude)



	def compare_position_to_flags(self, current_utm):
		"""
		Loops through flags and calculates distance from current
		position to the flags. If within some distance, publish on /at_flag
		topic to tell robot to stop!
		"""

		current_flag = self.flags[self.flag_index]  # grab current flag, format: [easting, northing]

		flag_distance = math.sqrt((current_utm[1] - current_flag[1])**2 + (current_utm[0] - current_flag[0])**2)

		print("Distance from flag {}: {}".format(self.flag_index, flag_distance))

		if flag_distance <= self.flag_tolerance:
			
			print("Robot has reached the flag within given tolerance!")
			print("Sending message to nav controller to stop the robot.")
			
			self.flag_publisher.publish(True)  # Publishes to drive routine to stop robot at the flag

			if self.flag_index >= len(self.flags) - 1:
				print(">>> Finished driving to flags list. <<<")
				print(">>> Continuing the rest of the row <<<")
				# self.flag_publisher.publish(False)
				self.flag_run_complete = True
				rospy.sleep(0.5)
				self.flag_publisher.publish(False)
				return

			self.flag_index += 1

		else:
			self.flag_publisher.publish(False)
		
		return



	def position_callback(self, current_fix):
		"""
		Position callback, which is executed in the event that a GPS fix is
		published by the Jackal.
		"""
		# print "jackal_pos_server: jackal's position: {}".format(current_fix)
		current_utm = self.get_utm_from_fix(current_fix)  # converts current fix to utm

		if not self.flag_run_complete and self.flag_index < len(self.flags):
			self.compare_position_to_flags(current_utm)

		return

		





if __name__ == '__main__':

	# _flag_filename = sys.argv[1]

	# flags_file = open(_flag_filename, 'r')
	# flags_data = flags_file.read()
	# flags_file.close()

	# flags_obj = json.loads(flags_data)

	# # Uses nav_tracks module to convert flags file to list of [easting, northing] pairs:
	# nt = nav_tracks.NavTracks()  # instantiates nav_tracks module
	# flags_array = nt.get_flags_from_geojson(flags_obj)  # returns list of [easting, northing] pairs

	try:
		flag_handler = FlagHandler()
	except rospy.ROSInterruptException:
		raise