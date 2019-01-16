"""
Navigation settings for the Jackal that are accessed
by the "robot-agnostic" main drive module, i.e., a single drive
node that drives the Jackal or the Red Rover.
"""

from general_settings import RobotSettings



class JackalSettings(RobotSettings):

	def __init__(self):

		RobotSettings.__init__(self)

		self.drive_speeds = {
			'slow': 0.1,
			'med': 0.3,
			'fast': 0.5
		}

		self.turn_speed = 0.1  # currently one turn speed (for jackal and red rover)

		self.ros_drive_topics = ['/cmd_vel']
		self.ros_turn_topics = ['/cmd_vel']

	def initial_turn_sequence(self):
		"""
		Velocity object used by Jackal for executing a turn.
		"""
		return Twist()

	def execute_turn(self):
		"""
		"""
		pass

	def final_turn_sequence(self):
		"""
		"""



# class NavController(object):
# 	"""
# 	Handles proportional controls for various navigation
# 	parameters for the Jackal.

# 	Current control parameters: angle trim and linear speed, which
# 	are both proportional to the Jackal's turn angle.
# 	"""

# 	def __init__(self):

# 		self.min_angle = 0.01  # min possible turn angle, any lower is considered straight
# 		self.max_angle = 360  # max possible turn angle

# 		self.min_trim_angle = 0.1  # min trim angle when turn angle is smallest
# 		self.max_trim_angle = 20.0  # max trim angle when turn angle is largest

# 		self.min_linear_speed = 0.1  # minimum linear speed
# 		self.max_linear_speed = 0.5  # maximum linear speed

# 		self.kp = 5.0  # proportional constant for nav controller
# 		self.sp = 0.1  # set point for nav controller (e.g., trim angle)

# 		# Trim angle slope:
# 		self.angle_to_trim_slope = (self.max_trim_angle - self.min_trim_angle) / (self.max_angle - self.min_angle)

# 		# Trim angle y-intercept:
# 		self.angle_to_trim_intercept = self.min_trim_angle - self.angle_to_trim_slope * self.min_angle 





# 	def angle_to_trim_angle(self, turn_angle):
# 		"""
# 		Proportional control function for trim angle.
# 		"""
# 		return self.angle_to_trim_slope * turn_angle + self.angle_to_trim_intercept



# 	def angle_to_linear_speed(self, turn_angle):
# 		"""
# 		Proportional control function for linear speed.
# 		Initially making this a fuzzy controller instead
# 		of continuous like the trim angle one.

# 		Inputs: turn_angle - expecting value b/w 0 - 360
# 		"""
# 		new_linear_speed = None

# 		if turn_angle < self.min_angle:
# 			print("turn_angle out of range: {}".format(turn_angle))
# 			return None

# 		if turn_angle > self.max_angle:
# 			print("turn_angle out of range: {}".format(turn_angle))
# 			return None

# 		if turn_angle >= self.min_angle and turn_angle < 3.0:
# 			new_linear_speed = self.max_linear_speed
# 		elif turn_angle >= 3.0 and turn_angle < 30.0:
# 			new_linear_speed = 0.3
# 		elif turn_angle >= 30.0:
# 			new_linear_speed = self.min_linear_speed
# 		else:
# 			new_linear_speed = None

# 		return new_linear_speed