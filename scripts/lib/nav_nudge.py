import numpy as np
import sys
import math
import matplotlib.pyplot as plt
import json



class NavNudge(object):

    def __init__(self, course_data, nudge_factor, space_factor):

        self.course_data = course_data
        self.nudge_factor = nudge_factor
        self.space_factor = space_factor

        # Assuming course is in JSON format like previous courses (see /courses folder in this package):
        self.course_data = self.build_array_from_json()  # loads course_data as json object, picks out utm values

        self.course_data_parsed = self.parse_by_space_factor()  # parses course points before calculating parallel nudge line

        self.nudged_course = self.offset(self.course_data_parsed, self.nudge_factor)



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



    def plot_results(self):

        x1 = [x[0] for x in self.course_data]  # all x vals in original course
        y1 = [x[1] for x in self.course_data]  # all y vals in original course

        x2 = [x[0] for x in self.nudged_course]
        y2 = [x[1] for x in self.nudged_course]

        plt.plot(x1, y1, 'g', x2, y2, 'b', lw=1)
        plt.plot(x1, y1, 'go', x2, y2, 'bo')

        plt.grid(True)

        plt.axes().set_aspect('equal', 'datalim')

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

   
    # # Opens course file:
    # with open(course_filename) as f:
    #     f.next()
    #     data = [[float(x) for x in line.split(',')] for line in f if line.strip()]
    file_data = open(course_filename, 'r').read()


    nav_nudge = NavNudge(file_data, nudge_factor, space_factor)
    nav_nudge.plot_results()