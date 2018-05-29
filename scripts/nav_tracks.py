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

		# self.track1 = [[0,0],[1,1]]  # simple test of 1m x 1m
		self.track1 = [[1,1], [2,2]]

		self.track2 = [[259740.58617625054, 3485057.3168547815]]  # single flag goal behind annex

		self.track3 = [[259742.0772760457, 3485056.984245597]]  # First flag from 4-5-18 eng. annex test row 1

		self.track4 = [[259742.1089532437, 3485056.983515083],  # Set of goals from 4-5-18 eng. annex test row
						[259739.9971399921, 3485057.0322162253],
						[259737.0933967415, 3485057.0991809973],
						[259733.65459685936, 3485056.8703144793],
						[259729.15988999663, 3485056.665800806],
						[259726.22773249005, 3485055.5007375698],
						[259721.68329927392, 3485053.1401727293],
						[259717.40284074377, 3485050.7735215155],
						[259713.1223801818, 3485048.406871946]]


	def get_track(self, track):

		for key,val in self.__dict__.items():
			if key == track:
				return val


	def get_track_from_course(self, course):
		"""
		Takes position file in format of goals, and grabs the
		UTM positions to build a list of UTM x,y positions (e.g., 
		the hard-coded tracks in the __init__ function)
		"""

		goals = course.get('goals')  # get list of goals
		track_list = []  # track list from course positions

		if len(goals) <= 0:
			raise "Could not find goals in course object, make_track_from_course function, nav_tracks module.."

		for goal in goals:
			_easting = goal.get('utmPos', {}).get('easting')
			_northing = goal.get('utmPos', {}).get('northing') 
			track_list.append([_easting, _northing])  # building list of [easting, northing] objects

		return track_list

