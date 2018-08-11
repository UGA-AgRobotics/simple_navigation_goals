import dubins
import math
import json
import math
import sys
import numpy as np
import matplotlib.pyplot as plt
import orientation_transforms as ot  # local requirement



def plot_dubins_path(qs, q0, q1, show=True):
	"""
	Plots the dubins path between a starting and ending point.
	Inputs:
		qs - dubins path data [[x,y,angle], ..]
		q0 - initial position [x,y,angle]
		q1 - target position [x,y,angle]
	Returns: None
	"""
	xs = qs[:,0]
	ys = qs[:,1]
	us = xs + np.cos(qs[:, 2])
	vs = ys + np.sin(qs[:, 2])
	plt.plot(xs, ys, 'b-')
	plt.plot(xs, ys, 'r.')
	plt.plot(qs[0][0], qs[0][1], 'go', markersize=5)  # dubins start point
	plt.plot(qs[-1][0], qs[-1][1], 'ro', markersize=5)  # dubins end point

	# plots velocity vectors:
	for i in xrange(qs.shape[0]):
		plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')

	if show:
		plt.show()


def plot_full_dubins_path(qs_array, x_path, y_path):
	"""
	Like plot_dubins_path() function, but plots a full set of points
	instead a single A -> B two point dataset.
	"""
	# Initial setup: No directional plotting, just dots and path at the moment..
	for qs in qs_array:
		plot_dubins_path(qs['qs'], qs['q0'], qs['q1'], show=False)
	plt.plot(x_path, y_path, 'bo')  # overlay path points onto plot
	plt.show()  # display plot



def get_rover_imu_orientation(img_angle):

	angle = ot.transform_imu_frame(math.degrees(imu_angle))
	print("rover imu angle: {}".format(angle))
	return angle


def get_row_angle(row_array):

	first_pos = row_array[0]  # first recorded position in course
	last_pos = row_array[-1]  # last recorded position in course
	row_angle = math.atan2((last_pos[1] - first_pos[1]), (last_pos[0] - first_pos[0]))  # computer angle in imu frame
	print("row angle (radians): {}".format(row_angle))
	return row_angle



def determine_dubins_AB_points(exit_row, entry_row, exit_row_angle, entry_row_angle, angle_diff, angle_tolerance):
	"""
	Determines Dubins point A and B.
	Currently uses last point in exit row as A, and last or first
	point in entry row as B, but doesn't account for flipping the row
	based on course and rover direction.
	"""

	q0 = (exit_row[-1][0], exit_row[-1][1], exit_row_angle)  # dubins point A

	if angle_diff < math.pi + angle_tolerance and angle_diff > math.pi - angle_tolerance:
		# entry row is recorded in opposite direction as exit row
		q1 = (entry_row[0][0], entry_row[0][1], entry_row_angle)  # dubins point B

	elif angle_diff < 0.0 + angle_tolerance and angle_diff > 0.0 - angle_tolerance:
		# entry row is recorded in same direction as exit row
		q1 = (entry_row[-1][0], entry_row[-1][1], entry_row_angle + math.pi)  # dubins point B, flipped entry angle

	return q0, q1



def plot_handler(configurations, exit_row, entry_row, q0, q1, show=True):

	dubins_course = np.array(configurations)

	plot_dubins_path(dubins_course, q0, q1, False)  # creates plot for dubins exit -> entry rows

	# plots rows along with dubins path:
	exit_row = np.array(exit_row)
	entry_row = np.array(entry_row)
	plt.plot(exit_row[:,0], exit_row[:,1], 'bo')
	plt.plot(entry_row[:,0], entry_row[:,1], 'bo')

	if show:
		plt.show()



def handle_dubins(course_data, exit_row_index, entry_row_index, angle_tolerance=0.523599):

	# Dubins parameters:
	turning_radius = 1.5
	step_size = 0.5

	exit_row, entry_row = [], []

	# Gets start and end rows arrays:
	for row_obj in course_data['rows']:

		if int(row_obj['index']) == int(exit_row_index):
			exit_row = row_obj['row']

		elif int(row_obj['index']) == int(entry_row_index):
			entry_row = row_obj['row']

	print("exit_row_index: {}".format(exit_row_index))
	print("entry_row_index: {}".format(entry_row_index))


	# Determines angle of rows in IMU frame:
	exit_row_angle = get_row_angle(exit_row)
	entry_row_angle = get_row_angle(entry_row)
	print("Exit row angle: {}".format(math.degrees(exit_row_angle)))
	print("Entry row angle: {}".format(math.degrees(entry_row_angle)))

	# Calculates row spacing at exit/entry points:
	distance_apart = math.sqrt( (exit_row[-1][0] - entry_row[0][0])**2 + (exit_row[-1][1] - entry_row[0][1])**2 )
	print("Linear distance between exit and entry rows: {} meters".format(distance_apart))

	# Determines which end of row to use based on row angles:
	angle_diff = abs(exit_row_angle - entry_row_angle)
	print("Angle diff b/w rows {}".format(math.degrees(angle_diff)))


	# TODO: Use angle between rover and entry row.
	
	# TODO: Use rover position for A/q0 or keep A/q0 as last point in row so
	# it doesn't start turning while it's still in the row.



	# 




	# 	
	q0, q1 = determine_dubins_AB_points(exit_row, entry_row, exit_row_angle, entry_row_angle, angle_diff, angle_tolerance)

	path = dubins.shortest_path(q0, q1, turning_radius)
	configurations, _ = path.sample_many(step_size)

	plot_handler(configurations, exit_row, entry_row, q0, q1)

	dubins_course = np.array(configurations)

	return dubins_course






if __name__ == '__main__':

	x0, y0, theta0 = 0, 0, 0
	x1, y1, theta1 = 1, 1, math.pi
	angle_tolerance = math.radians(30)

	input_filename = sys.argv[1]
	exit_row_index = int(sys.argv[2])  # row the rover is exiting
	entry_row_index = int(sys.argv[3])  # row it's about to go down

	# open field data array:
	filein = open(input_filename, 'r')
	course_data = json.loads(filein.read())
	filein.close()

	handle_dubins(course_data, exit_row_index, entry_row_index, angle_tolerance)