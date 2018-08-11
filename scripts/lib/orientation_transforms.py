#!/usr/bin/env python

"""
Rover-related angle transformations between rover IMU and world frame.
"""
from math import atan2, degrees



def determine_angle_at_goal(goal, future_goal):
	"""
	Calculates the angle of orientation for the robot to be in when
	it gets to its goal. Creates a vector that points to the future goal.
	If there is no future goal, end in the same orientation, just end at 
	the goal in the orientation it arrives in.
	"""
	if not future_goal:
		return None

	x_diff = future_goal[0] - goal[0]
	y_diff = future_goal[1] - goal[1]

	return atan2(y_diff, x_diff)  # returns full angle relative to easting,north -> x,y axis



def transform_angle_by_quadrant(initial_angle, x_diff, y_diff):
	"""
	Takes the change in X and Y to determine how the Jackal
	should turn.
	"""
	if x_diff > 0 and y_diff > 0:
		print("p1 in quadrant: {}".format(1))
		# Point B in quadrant 1..
		return degrees(initial_angle)
	elif x_diff < 0 and y_diff > 0:
		print("p1 in quadrant: {}".format(2))
		# Point B in quadrant 2..
		return 180 - degrees(initial_angle)
	elif x_diff < 0 and y_diff < 0:
		print("p1 in quadrant: {}".format(3))
		# Point B in quadrant 3..
		return 180 + degrees(initial_angle)
	elif x_diff > 0 and y_diff < 0:
		print("p1 in quadrant: {}".format(4))
		# Point B in quadrant 4..
		return 360 - degrees(initial_angle)
	elif x_diff == 0 and y_diff == 0:
		# No change in angle..
		return 0.0
	else:
		raise



def transform_imu_frame(theta0):
	"""
	Transform initial robot angle in IMU frame to have 0 degrees
	in East direction, going 0->360 CCW.
	"""
	_trans = theta0 + 90  # shift value 90 degrees from N->E

	if _trans > -180 and _trans < 0:
		_trans = 360 + _trans  # transform angle to 0->360 if in -180->0 quadrants

	return _trans



def initiate_angle_transform(A, B):
	"""
	Transforms angle between the IMU frame (magnetic North) and
	the Jackal's frame. Takes in angle from IMU, then determines
	an angle and turn direction for the Jackal to execute.
	"""
	x_diff = B[0] - A[0]
	y_diff = B[1] - A[1]

	_trans_angle = transform_imu_frame(degrees(A[2]))
	AB_theta0 = atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
	AB_angle = transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B

	turn_angle = None
	if AB_angle == 0:
		turn_angle = 0
	else:
		turn_angle = AB_angle - _trans_angle  # angle to turn (signage should denote direction to turn)

	return turn_angle  # return angle to turn relative to jackal's current orientation