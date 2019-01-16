#!/usr/bin/env python

import rospy
import subprocess
import os
import sys
import time
from std_msgs.msg import Bool, String

# Local requirements:
from lib.course_file_handler import CourseFileHandler


class RosbagRecord(object):

    # node_name = "rosbag_record"

    # def __init__(self, rosbag_script, rosbag_folder, rosbag_output_filename="rosbag_fix_data"):
    def __init__(self):

        self.course_file_handler = CourseFileHandler()  # instance of CourseFileHandler class

        self.node_name = "rosbag_manager"  # this node's name
        self.rosbag_script_node_name = "rosbag_record"  # the rosbag node from the script command

        self.is_recording = False

        self.record_script = "rosbag_record_path_script.sh"  # filename of the rosbag record sh file
        self.record_folder = "./lib"  # location of rosbag record sh file
        self.record_output_filename = ""

        # ROS node settings:
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.terminate_ros_node)

        # Subscribers:
        # rospy.Subscriber("/start_recording", Bool, self.start_recording_callback, queue_size=1)
        rospy.Subscriber("/start_recording", String, self.start_recording_callback, queue_size=30)

        # Wait for shutdown signal to close rosbag record
        rospy.spin()




    def start_recording_callback(self, message):
        """
        Starts recording a rosbag of /fix data until
        it receives a False to stop recording.
        """

        print("Received message at start_recording_callback: {}".format(message))

        if not message.data:
            return

        if message.data != "stop" and not self.is_recording:
            self.is_recording = True
            self.record_output_filename = message.data  # sets message.data as rosbag filename
            self.start_ros_node()
        elif (message.data == "stop" or not message.data) and self.is_recording:
            self.is_recording = False
            self.terminate_ros_node()

            rospy.sleep(1)  # provides some time for rosbag to be created

            # TODO: Run code to convert /fix bag file to a course/path file (single vs multi row?)
            self.create_path_file_from_rosbag(self.record_output_filename)

        else:
            pass



    def start_ros_node(self):
        """
        Starts rosbag recording script as a ROS node.
        Note: Rosbag output filename has timestamp appended
        to it, e.g., rosbag_fix_data_20180515-155045.
        """
        # Command that launches rosbag record command with a node name,
        # which enables it to be stopped with 'rosnode kill' command:
        command = "source " + self.record_script + " " \
                    + self.record_output_filename + " " \
                    + self.rosbag_script_node_name

        self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                                  executable='/bin/bash')



    def terminate_ros_node(self):
        """
        Executes some command line arguments to find the
        rosbag ROS node, then kills it.
        """

        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()

        assert retcode == 0, "List command returned %d" % retcode

        # Searches for rosbag node, then kills it:
        for msg in list_output.split("\n"):
            if (msg.startswith("/" + self.rosbag_script_node_name)):
                os.system("rosnode kill " + msg)



    def create_path_file_from_rosbag(self, rosbag_filename):
        """
        Creates a path file from the /fix rosbag data.
        """
        # self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
        #                           executable='/bin/bash')

        rospack_path = self.get_rospack_path()  # full path of ros package

        input_filename = rospack_path + '/courses/' + rosbag_filename + '.bag'  # assuming rosbag is in courses directory
        output_filename = rospack_path + '/courses/' + rosbag_filename + '.json'

        # Converts /fix rosbag into path file for rover to follow:
        self.course_file_handler.main(3, input_filename, output_filename)



    def get_rospack_path(self):
        record_node_location = os.environ.get('ROS_PACKAGE_PATH')
        if not record_node_location:
            raise Exception("No ROS_PACKAGE_PATH envivornment variable.")

        command_loc = record_node_location.split(':')[0]  # structure example: /home/nick/ros_workspaces/simple_nav_goals_workspace/src:/opt/ros/kinetic/share
        command_loc = command_loc + "/simple_navigation_goals"

        print("Module location: {}".format(command_loc))

        return command_loc



    def generate_output_filename(self):
        """
        Creates a timestamp for rosbag output filename.
        """
        return self.record_output_filename + "_" +  time.strftime("%Y%m%d-%H%M%S")



if __name__ == '__main__':

    # rosbag_script = sys.argv[1]
    # rosbag_folder = sys.argv[2]
    # rosbag_start_recording = False

    # try:
    #     rosbag_start_recording = bool(sys.argv[3])
    # except Exception:
    #     pass

    # Go to class functions that do all the heavy lifting. Do error checking.
    # try:
    # rosbag_record = RosbagRecord(rosbag_script, rosbag_folder)
    # except rospy.ROSInterruptException as e:
    #     print("Exception running rosbag_record node: {}".format(e))
    #     pass

    try:
        rosbag_record = RosbagRecord()
    except rospy.ROSInterruptException as e:
        print("Exception running rosbag_record node: {}".format(e))
        pass