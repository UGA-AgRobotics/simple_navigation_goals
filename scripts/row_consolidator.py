"""
Combines all the separate row files into one
file using the same JSON format as the row files.
"""

import sys
import json
import utm



def convert_latlon_csv_to_course_array(input_filename, n_skip=1):
	"""
	Converts a CSV of lat, lons to a JSON formatted course.
	"""
	print("Opening course file..")
	filein= open(input_filename, 'r')
	file_data = filein.read()
	filein.close()

	latlon_pairs = file_data.split('\n')

	course_array = []

	print("Lat/lons: {}".format(file_data))		
	print("Building course file from lat, lons..")
	
	for i in range(0, len(latlon_pairs) - 1, n_skip):
		
		_lat = float(latlon_pairs[i].split(',')[0])
		_lon = float(latlon_pairs[i].split(',')[1])

		utm_val = utm.from_latlon(_lat, _lon)
		easting, northing = utm_val[0], utm_val[1]
		course_array.append([easting, northing])

	return course_array



def convert_rowfiles_to_course_array(input_filename, n_skip=1):
	"""
	Converts a set of course JSON files consisting of single rows,
	and puts them into one array that's indexed by row.
	Ex: {'date': "", 'location': "", 'rows': [{'index': 1, 'flags': [[x1, y1],..], 'row': [[x1,y1],..]}]}
	"""
	
	# opens course file:
	print("Opening course file..")
	filein= open(input_filename, 'r')
	file_data = filein.read()
	filein.close()

	row_obj = json.loads(file_data)  # get row data
	row_data = row_obj['flags']  # get array of positions

	result_array = []

	# for pos_obj in row_data:
	for i in range(0, len(row_data) - 1, n_skip):

		easting = float(row_data[i]['utmPos']['easting'])  # gets easting utm value
		northing = float(row_data[i]['utmPos']['northing'])  # gets northing utm value

		result_array.append([easting, northing])

	return result_array






if __name__ == '__main__':

	num_files = int(sys.argv[1])
	output_filename = sys.argv[2]
	n_skip = int(sys.argv[3])

	field_data = {
		'name': "Peanut Field 2018",
		'date': "July 2018",
		'rows': []
	}

	for i in range(1, num_files + 1):

		# input_filename = "../courses/peanut_field_2018/row_{}_latlons.csv".format(i)
		input_filename = "../courses/course_{}_filled.json".format(i + 12)
		print("Opening {}".format(input_filename))

		row_obj = {
			'index': str(i),
			'row': []
		}

		# row_obj['row'] = convert_latlon_csv_to_course_array(input_filename, n_skip)  # get row data
		row_obj['row'] = convert_rowfiles_to_course_array(input_filename, n_skip)

		field_data['rows'].append(row_obj)

	fileout = open(output_filename, 'w')
	fileout.write(json.dumps(field_data))
	fileout.close()

	print("done.")