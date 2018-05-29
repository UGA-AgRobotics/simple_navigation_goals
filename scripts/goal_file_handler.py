#!/usr/bin/env python

"""
Python module for reading in goal files and adding any format that's
not already there.

Goal file has the following keys: 'date', 'location', 'units', 'goals'.
Goals can be in the following units: 'dsm', 'dec', 'utm'.

This module finds the format currently used, and adds the remaining formats to
the goals file.
"""

import utm
import json
import sys



class GoalFileHandler(object):
	"""
	Handles goal/position data using the keys laid out below.
	See UGA-AgRobotics/RoverWatch/goals for examples of goal files.
	"""

	def __init__(self):

		self.goals = None  # goals json object from goals file

		self.units = ['dsm', 'dec', 'utm']

		self.dsm_object = {
			'deg': None,
			'min': None,
			'sec': None
		}
		self.dsm_pos = {
			'lat': self.dsm_object,
			'lon': self.dsm_object
		}

		self.dec_pos = {
			'lat': None,
			'lon': None
		}

		self.utm_pos = {
			'easting': None,
			'northing': None,
			'zone': None,
			'letter': None
		}

		self.goal_object = {
			'index': None,
			'dsm_pos': None,
			'dec_pos': None,
			'utm_pos': None
		}



	def read_goals_file(self, filename):
		"""
		Reads in goals JSON file.
		"""
		goals_data = None
		with open(filename, 'r') as json_data:
			goals_data = json.loads(json_data.read())

		self.goals = goals_data
		print("Goals data set: {}".format(self.goals))



	def save_goals_file(self, filename, updated_goals):
		"""
		Save updated goals file.
		"""
		with open(filename, 'w') as fileout:
			fileout.write(json.dumps(updated_goals))


	def convert_dsm_to_dec(self, dsm_pos):
		"""
		Input lat/lon format: [degree, minute, second]
		"""
		# new_pos = self.dec_pos()
		new_pos = {}
		# new_lat, new_lon = None, None
		dec_pos = None

		for key, val in dsm_pos.items():
			# looping lat/lon key:vals
			if val.get('deg') < 0:
				dec_pos = dsm_pos[key]['deg'] - dsm_pos[key]['min']/60.0 - dsm_pos[key]['sec']/3600.0
			else:
				dec_pos = dsm_pos[key]['deg'] + dsm_pos[key]['min']/60.0 + dsm_pos[key]['sec']/3600.0
			
			new_pos[key] = dec_pos

		return new_pos



	def convert_dec_to_dsm(self, dec_pos):
		"""
		Inputs: lat/lon are single decimal numbers.
		Returns: [lat,lon], where lat/lon = [deg, min, sec]
		"""
		# new_pos = self.dmsPos
		new_pos = {}
		for key, val in dec_pos.items():
			new_pos[key] = {
				'deg': int(val),
				'min': int((val % 1) * 60.0),
				'sec': (((val % 1) * 60.0) % 1) * 60.0
			}
		return new_pos



	def convert_dec_to_utm(self, dec_pos):
		"""
		Converts decimal lat/lon to UTM format.
		"""
		new_pos = self.utm_pos
		utm_val = utm.from_latlon(dec_pos['lat'], dec_pos['lon'])
		new_pos = {
			'easting': utm_val[0],
			'northing': utm_val[1],
			'zone': utm_val[2],
			'letter': utm_val[3]
		}
		return new_pos



	def convert_utm_to_dec(self, utm_pos):
		"""
		Converts UTM position to decimal lat/lon format.
		"""
		new_pos = self.dec_pos
		dec_val = utm.to_latlon(utm_pos['easting'], utm_pos['northing'], utm_pos['zone'], utm_pos['letter'])
		new_pos = {
			'lat': dec_val[0],
			'lon': dec_val[1]
		}
		return new_pos



	def fill_out_goals_file(self):
		"""
		Loops through goals JSON and fills in any formats
		that aren't currently there.
		"""
		if not self.goals:
			raise "No goals specificed. Run read_goals_file(filename) first.."

		if not self.goals.get('units'):
			raise "Goals file needs a 'units' key:val of 'dsm', 'dec', or 'utm'"

		goal_date = self.goals.get('date')
		goal_location = self.goals.get('location')
		goal_units = self.goals.get('units')
		goals = self.goals.get('goals')

		for goal in goals:

			if goal_units == 'dsm':
				# Goals file has DSM lat/lons, add dec and utm..
				goal['decPos'] = self.convert_dsm_to_dec(goal['dsmPos'])
				goal['utmPos'] = self.convert_dec_to_utm(goal['decPos'])  # UTM conversion uses dec lat/lons


			elif goal_units == 'dec':
				# Goals file has decimal lat/lons, add dsm and utm..
				goal['dsmPos'] = self.convert_dec_to_dsm(goal['decPos'])
				goal['utmPos'] = self.convert_dec_to_utm(goal['decPos'])

			elif goal_units == 'utm':
				# Goals file has UTM positions, add dec and dsm..
				goal['decPos'] = self.convert_utm_to_dec(goal['utmPos'])
				goal['dsmPos'] = self.convert_dec_to_dsm(goal['decPos'])

		return goals  # returning updated goals



	def parse_bag_data_to_goals(self):
		"""
		Takes /fix data from bag file handler (e.g., {"topic": "/fix", "data": [{"lat": "", "lon": ""}]}),
		and parses it to the goal JSON format.
		"""
		bag_data_list = self.goals
		parsed_results = {'goals': [], 'units': "dec"}  # assuming /fix topic is dec lat/lon..
		
		if len(bag_data_list) == 1:
			# assuming /fix data is all that's in bag data if length is 1..
			bag_data_list = self.goals[0].get('data')  # get list of {'lat': "", 'lon': ""} objects
		else:
			raise "More than one topic in bag data.. TODO: loop to find desired topic instead of raising exception.."

		_index = 1
		for _pos in bag_data_list:
			parsed_data_obj = {
				'index': _index,
				'decPos': {
					'lat': _pos.get('lat'),
					'lon': _pos.get('lon')
				}
			}
			parsed_results['goals'].append(parsed_data_obj)
			_index += 1

		self.goals = parsed_results  # set goals to parsed results




if __name__ == '__main__':
	"""
	Fills out goals JSON to have all formats of position (UTM, lat/lon deg and dsm).

	Inputs: 
		1. filename - name of initial goals file.
		2. is_bag_file - whether it's position data from a bag file or not (bag_handler.py)

	Output: A parsed JSON file named filename_updated.json
	"""

	filename = None
	is_bag_file = False

	try:
		filename = sys.argv[1]  # get filename from command line
	except IndexError as e:
		raise "Must add an input filename for goals file!"

	try:
		is_bag_file = sys.argv[2]
		if is_bag_file in ["true", "True", True]:
			is_bag_file = True
	except IndexError as e:
		print "Didn't provide bool for data being from bag file or not, assuming it's not from bag_handler.py.."

	filename_out = "{}_updated.json".format(filename.split('.')[0])

	goal_file_handler = GoalFileHandler()

	print("Reading in goals file: {}".format(filename))

	goal_file_handler.read_goals_file(filename)

	print("Goals file read.")
	print("Filling out goals file with missing formats..")

	if is_bag_file:
		# Parses bag_handler data to goals file format before filling out position data..
		goal_file_handler.parse_bag_data_to_goals()

	updated_goals = goal_file_handler.fill_out_goals_file()
	goal_file_handler.goals['goals'] = updated_goals

	print("Goals file now updated.")
	print("Saving updated goals file as: {}..".format(filename_out))

	goal_file_handler.save_goals_file(filename_out, goal_file_handler.goals)

	print("Done.")