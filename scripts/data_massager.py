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



class DataMassager(object):

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
		new_pos = self.dec_pos
		new_pos = {
			'lat': dsm_pos['lat']['deg'] + dsm_pos['lat']['min']/60.0 + dsm_pos['lat']['sec']/3600.0,
			'lon': dsm_pos['lon']['deg'] + dsm_pos['lon']['min']/60.0 + dsm_pos['lon']['sec']/3600.0
		}
		
		return new_pos



	def convert_dec_to_dsm(self, dec_pos):
		"""
		Inputs: lat/lon are single decimal numbers.
		Returns: [lat,lon], where lat/lon = [deg, min, sec]
		"""
		new_pos = self.dmsPos
		for key, val in dec_pos:
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







if __name__ == '__main__':
	# Testing procedure follows:
	filename = sys.argv[1]  # get filename from command line
	filename_out = "{}_updated.json".format(filename.split('.')[0])
	datmass = DataMassager()
	print("Reading in goals file: {}".format(filename))
	datmass.read_goals_file(filename)
	print("Goals file read.")
	print("Filling out goals file with missing formats..")
	updated_goals = datmass.fill_out_goals_file()
	print("Goals file now updated.")
	print("Saving updated goals file as: {}..".format(filename_out))
	datmass.save_goals_file(filename_out, updated_goals)
	print("Done.")