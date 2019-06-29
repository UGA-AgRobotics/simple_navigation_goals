"""
Testing PI(D) implementation for the rover's navigation.
"""

import matplotlib.pyplot as plt



class NavController(object):
	"""
	Handles proportional controls for various navigation
	parameters for the Jackal.

	Current control parameters: angle trim and linear speed, which
	are both proportional to the Jackal's turn angle.
	"""

	def __init__(self):

		self.min_angle = 0.01  # min possible turn angle, any lower is considered straight
		self.max_angle = 360  # max possible turn angle

		self.min_trim_angle = 0.1  # min trim angle when turn angle is smallest
		self.max_trim_angle = 50.0  # max trim angle when turn angle is largest

		self.min_linear_speed = 0.1  # minimum linear speed
		self.max_linear_speed = 0.5  # maximum linear speed

		self.kp = 5.0  # proportional constant for nav controller
		self.sp = 0.1  # set point for nav controller (e.g., trim angle)

		# Trim angle slope:
		self.angle_to_trim_slope = (self.max_trim_angle - self.min_trim_angle) / (self.max_angle - self.min_angle)

		# Trim angle y-intercept:
		self.angle_to_trim_intercept = self.min_trim_angle - self.angle_to_trim_slope * self.min_angle 





	def angle_to_trim_angle(self, turn_angle):
		"""
		Proportional control function for trim angle.
		"""
		return self.angle_to_trim_slope * turn_angle + self.angle_to_trim_intercept



	def angle_to_linear_speed(self, turn_angle):
		"""
		Proportional control function for linear speed.
		Initially making this a fuzzy controller instead
		of continuous like the trim angle one.

		Inputs: turn_angle - expecting value b/w 0 - 360
		"""
		new_linear_speed = None

		if turn_angle < self.min_angle:
			print("turn_angle out of range: {}".format(turn_angle))
			return None

		if turn_angle > self.max_angle:
			print("turn_angle out of range: {}".format(turn_angle))
			return None

		if turn_angle >= self.min_angle and turn_angle < 3.0:
			new_linear_speed = self.max_linear_speed
		elif turn_angle >= 3.0 and turn_angle < 20.0:
			new_linear_speed = 0.3
		elif turn_angle >= 20.0 and turn_angle < 40.0:
			new_linear_speed = 0.2
		elif turn_angle >= 40.0:
			new_linear_speed = self.min_linear_speed
		else:
			new_linear_speed = None

		return new_linear_speed






# for i in range(0, 36000, 1):

# 	angle = i / 100.0

# 	if angle < min_angle:
# 		continue

# 	f = angle_to_trim_equation(angle)
	
# 	# f = ((angle - min_angle) / (max_angle - min_angle)) + min_angle
# 	# f = ((angle - min_angle_trim) / (max_angle_trim - min_angle_trim)) + min_angle_trim
	
# 	Pout = Kp * (SP - f)
	
# 	# x.append(i)
# 	x.append(angle)
# 	# y.append(f)
# 	y.append(Pout)
# 	Kp_array.append(Kp)
# 	SP_array.append(SP)

# plt.plot(x, y, 'ro')
# # plt.plot(x, Kp_array, 'bo')
# # plt.plot(x, SP_array, 'go')
# plt.show()