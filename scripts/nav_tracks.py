#!/usr/bin/env python

"""
Python module for handling tracks/courses for the robot
to follow.

A track is a series of points (e.g., x,y; lat,lon; easting,northing)
that are intended for the robot to follow.
"""



class NavTracks(object):

	def __init__(self):

		self.name = ''  # name of track, e.g., track1
		self.units = ''  # units of points (e.g., utm)
		self.track_data = []  # list of list, e.g., [[x0,y0],[x1,y1]]

		self.track1 = [[0,0],[1,1]]  # simple test of 1m x 1m

		self.track2 = [[259740.58617625054, 3485057.3168547815]]  # single flag goal behind annex


	def get_track(self, track):

		for key,val in self.__dict__.items():
			if key == track:
				return val