#!/usr/bin/env python

"""
Handles code associated with plotting rover model trajectories as well
as plotting the projected dubins paths from A->B points.
"""

import numpy as np
import matplotlib.pyplot as plt



def plot_dubins_path(self, qs, q0, q1, show=True):
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
	plt.plot(q0[0], q0[1], 'gx', markeredgewidth=4, markersize=10)  # actual start point
	plt.plot(q1[0], q1[1], 'rx', markeredgewidth=4, markersize=10)  # actual end point
	plt.plot(xs, ys, 'b-')
	plt.plot(xs, ys, 'r.')
	plt.plot(qs[0][0], qs[0][1], 'go', markersize=5)  # dubins start point
	plt.plot(qs[-1][0], qs[-1][1], 'ro', markersize=5)  # dubins end point

	for i in xrange(qs.shape[0]):
		plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')

	if show:
		plt.show()



def plot_full_dubins_path(self, qs_array, x_path=None, y_path=None, show=True, filename=None):
	"""
	Like plot_dubins_path() function, but plots a full set of points
	instead a single A -> B two point dataset.
	"""
	# Initial setup: No directional plotting, just dots and path at the moment..

	for qs in qs_array:
		self.plot_dubins_path(qs['qs'], qs['q0'], qs['q1'], show=False)

	plt.plot(x_path, y_path, 'bo')  # overlay path points onto plot

	if not show and filename:
		print("Saving plot named {}".format(filename))
		plt.savefig('{}.png'.format(filename))  # for plots across look-aheads
		print("Plot saved.")
	else:
		plt.show()  # display plot