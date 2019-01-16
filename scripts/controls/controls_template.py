"""
Skeleton class for robot action modules.
"""

import rospy
from geometry_msgs.msg import Twist



class RobotActions(TurnActions, DriveActions):

	def __init__(self):

		self.turn = None
		self.drive = None
		self.stop = None

		self.path_type = None
		self.path_types = ['single', 'multirow']

		self.follow_row = bool  # following a field row
		self.traverse_row = bool  # turning around to a new row

	def initial_turn_sequence(self):
		"""
		Any action to take right before executing a turn.
		Example: move_cmd = Twist(), or _turn_val = self.no_turn_val.
		"""
		pass

	def execute_turn(self):
		"""
		Function for handling robot turn.
		"""
		pass

	def final_turn_sequence(self):
		"""
		Any action to take after executing a turn.
		"""
		pass

	def start_driving(self):
		"""
		Function for handling drive initiation.
		"""
		pass

	def continue_driving(self):
		"""
		Function for handling the drive commands during
		the main path following loop.
		"""
		pass

	def stop_driving(self):
		"""
		Function for handling a robot stopping.
		"""
		pass




class TurnActions(object):
	"""
	Handles turn actions attributes for robots.
	"""

	def __init__(self):

		self.turn_left = None
		self.turn_right = None
		self.angular_speeds = []

		self.ros_turn_topics = []



class DriveActions(object):
	"""
	Handles drive actions and attributes for robots.
	"""

	def __init__(self):

		self.speed_settings: {
			'stop': None
			'slow': None,
			'med': None,
			'fast': None
		}

		self.ros_drive_topics = []



class NavSensors(object):
	"""
	Navigation sensors for general robot (e.g., Jackal, Red Rover).
	"""

	def __init__(self):

		self.gps = None
		self.imu = None