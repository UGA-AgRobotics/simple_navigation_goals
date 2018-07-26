import course_file_handler  # local requirement
import sys
import json
import utm



cfh = course_file_handler.CourseFileHandler()



def convert_latlon_csv_to_course(input_filename, n_skip=1):
	"""
	Converts a CSV of lat, lons to a JSON formatted course.
	"""
	print("Opening course file..")
	filein= open(input_filename, 'r')
	file_data = filein.read()
	filein.close()

	# json_obj = {}
	# json_obj['date'] = ""
	# json_obj['location'] = ""
	# json_obj['units'] = "dec"
	# json_obj['flags'] = []

	latlon_pairs = file_data.split('\n')

	print("Lat/lons: {}".format(file_data))		

	print("Building course file from lat, lons..")
	for i in range(0, len(latlon_pairs) - 1, n_skip):
		_lat = float(latlon_pairs[i].split(',')[0])
		_lon = float(latlon_pairs[i].split(',')[1])
		_pos_obj = {
			'index': str(i),
			'decPos': {
				'lat': _lat,
				'lon': _lon
			}
		}
		json_obj['flags'].append(_pos_obj)

	cfh.flags = json_obj  # sets flags object to fill out remaining data
	cfh.fill_out_flags_file()  # fills out pos objects with dsm and utm formats
	return cfh.flags



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

		input_filename = "../courses/peanut_field_2018/row_{}_latlons.csv".format(i)

		row_obj = {
			'index': str(i),
			'row': []
		}
		# row_obj['row'] = convert_latlon_csv_to_course(input_filename, n_skip)  # get row data
		row_obj['row'] = convert_latlon_csv_to_course_array(input_filename, n_skip)  # get row data

		field_data['rows'].append(row_obj)

	fileout = open(output_filename, 'w')
	fileout.write(json.dumps(field_data))
	fileout.close()

	print("done.")