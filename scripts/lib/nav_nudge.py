import numpy as np
import sys
import math
import matplotlib.pyplot as plt
import json



class NavNudge(object):

    def __init__(self, course_data, nudge_factor, space_factor=None):
        """
        Reads in course data to create a parallel line that's shifted
        over some distance (nudge_factor, in meters). The course can also
        be thinned out by removing points based on a space_factor (in meters).
        """
        self.course_data = course_data
        self.nudge_factor = nudge_factor
        self.space_factor = space_factor

        # Example row object in multi-row course file:
        self.multirow_item = {
            'index': int,
            'row': []
        }

        # # Assuming course is in JSON format like previous courses (see /courses folder in this package):
        # self.course_data = self.build_array_from_json()  # loads course_data as json object, picks out utm values
        # self.course_data_parsed = self.parse_by_space_factor()  # parses course points before calculating parallel nudge line
        # self.nudged_course = self.offset(self.course_data_parsed, self.nudge_factor)



    def nudge_course_row(self):
        """
        Nudge routine for a single-row course file.
        (Assumes original JSON course file format.)
        """
        self.course_data = self.build_array_from_json()  # loads course_data as json object, picks out utm values
        self.course_data_parsed = self.parse_by_space_factor()  # parses course points before calculating parallel nudge line
        return self.offset(self.course_data_parsed, self.nudge_factor)



    def nudge_course_multirow(self):
        """
        Nudge routine for a row in a multi-row course file.
        (Assumes multi-row course file where rows are already arrays.)
        """
        self.course_data_parsed = self.parse_by_space_factor()
        return self.offset(self.course_data_parsed, self.nudge_factor)



    def offset(self, coordinates, distance):
        coordinates = iter(coordinates)
        x1, y1 = coordinates.next()
        z = distance
        points = []
        for x2, y2 in coordinates:
            # tangential slope approximation
            try:
                slope = (y2 - y1) / (x2 - x1)
                # perpendicular slope
                pslope = -1/slope  # (might be 1/slope depending on direction of travel)
            except ZeroDivisionError:
                continue
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2

            sign = ((pslope > 0) == (x1 > x2)) * 2 - 1

            delta_x = sign * z / ((1 + pslope**2)**0.5)
            delta_y = pslope * delta_x

            points.append([mid_x + delta_x, mid_y + delta_y])
            x1, y1 = x2, y2
        return points



    def parse_by_space_factor(self):

        parsed_array = []
        prev_xy = self.course_data[0]

        for i in range(1, len(self.course_data)):
            
            d = math.sqrt( (self.course_data[i][0] - prev_xy[0])**2 + (self.course_data[i][1] - prev_xy[1])**2 )

            if d > self.space_factor:
                parsed_array.append([self.course_data[i][0], self.course_data[i][1]])  # building array of course points spaced by space_factor..
                prev_xy = self.course_data[i]  # sets current x,y to prev for next iteration..

        return parsed_array



    def plot_results(self, course_data, nudged_course, show=True):

        x1 = [x[0] for x in course_data]  # all x vals in original course
        y1 = [x[1] for x in course_data]  # all y vals in original course

        x2 = [x[0] for x in nudged_course]
        y2 = [x[1] for x in nudged_course]

        plt.plot(x1, y1, 'g', x2, y2, 'b', lw=1)
        plt.plot(x1, y1, 'go', x2, y2, 'bo')

        plt.grid(True)

        plt.axes().set_aspect('equal', 'datalim')

        if show:
            plt.show()



    def build_array_from_json(self):
        """
        Builds array (list of easting,northing pairs) from course JSON.
        """
        course_json = json.loads(self.course_data)
        array_data = []

        for i in range(0, len(course_json.get('flags')) - 1):
            pos_obj = course_json['flags'][i]
            easting = pos_obj['utmPos']['easting']
            northing = pos_obj['utmPos']['northing']
            array_data.append([easting, northing])

        return array_data







if __name__ == '__main__':

    course_filename = sys.argv[1]  # filename/location of the course to nudge
    nudge_factor = float(sys.argv[2])  # amount of nudge/offset in meters
    space_factor = float(sys.argv[3])  # spacing between course points to use for calculating nudge


    if not '.json' in course_filename:
        raise Exception("Course input file must be .json format..")


    # Multi-row testing:
    #################################################################
    
    # 1. Testing single row by index
    ###########################################
    # try:
    #     row_index = int(sys.argv[4])
    # except IndexError as e:
    #     print("Warning: No index row, which is needed if running tests..")
    #     pass
    # file_data = json.loads(open(course_filename, 'r').read())
    # row_data = file_data['rows'][row_index]['row']  # get course array from multi-row data
    # nav_nudge = NavNudge(row_data, nudge_factor, space_factor)
    # nudged_course = nav_nudge.nudge_course_multirow()  # nudge row from multi-row data
    # nav_nudge.plot_results(row_data, nudged_course)
    ###########################################

    # 2. Testing all peanut field 2018 rows, flipping every other row,
    # then saves it as an output file (*_nudged.json)
    ###########################################
    file_data = json.loads(open(course_filename, 'r').read())

    nudged_rows = []

    i = 0
    for row_obj in file_data['rows']:

        nudged_obj = {}
        nudged_obj['index'] = i + 1

        if i % 2:
            # Flips row array every other row (to account for direction it was recorded)
            row_obj['row'] = [pos for pos in reversed(row_obj['row'])]

        nav_nudge = NavNudge(row_obj['row'], nudge_factor, space_factor)
        nudged_course = nav_nudge.nudge_course_multirow()
        nav_nudge.plot_results(row_obj['row'], nudged_course, False)

        nudged_obj['row'] = nudged_course
        nudged_rows.append(nudged_obj)

        i += 1

    plt.show()  # displays plot of original and nudged data

    # Creates output file that's same name as course with "_nudged" appended.
    # Example: course_1.json -> course_1_nudged.json

    fileout_name = course_filename.split('.json')
    if len(fileout_name) != 2:
        # Splits course filename, expects two items if '.json' file
        raise Exception("Expecting course input file to be of format 'coursename.json'..")
    fileout_name = fileout_name[0] + "_nudged.json"  # output filename built from course file name

    fileout_data = file_data.copy()  # using same data format as input course file
    fileout_data['rows'] = nudged_rows  # using nudged rows to create output file

    fileout = open(fileout_name, 'w')
    fileout.write(json.dumps(fileout_data))
    fileout.close()

    ###########################################

    #################################################################