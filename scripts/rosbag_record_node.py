#!/usr/bin/env python

import rospy
import subprocess
import os
import signal
import sys
from std_msgs.msg import Bool


class RosbagRecord(object):

    # node_name = "rosbag_record"

    def __init__(self, rosbag_script, rosbag_folder, start_recording=False, rosbag_output_filename="rosbag_data"):

        self.node_name = "rosbag_manager"  # this node's name
        self.rosbag_script_node_name = "rosbag_record"  # the rosbag node from the script command

        self.is_recording = False

        self.record_script = rosbag_script
        self.record_folder = rosbag_folder
        self.record_output_filename = rosbag_output_filename

        # ROS node settings:
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.terminate_ros_node)

        # Subscribers:
        rospy.Subscriber("/start_recording", Bool, self.start_recording_callback, queue_size=1)

        # Wait for shutdown signal to close rosbag record
        rospy.spin()




    def start_recording_callback(self, message):
        """
        Starts recording a rosbag of /fix data until
        it receives a False to stop recording.
        """

        if message.data == True and not self.is_recording:
            self.is_recording = True
            self.start_ros_node()
        elif message.data == False and self.is_recording:
            self.is_recording = False
            self.terminate_ros_node()
        else:
            pass



    def start_ros_node(self):
        """
        Starts rosbag recording script as a ROS node.
        """
        # Command that launches rosbag record command with a node name,
        # which enables it to be stopped with 'rosnode kill' command:
        command = "source " + self.record_script + " " \
                    + self.record_output_filename + " " \
                    + self.rosbag_script_node_name

        self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                                  executable='/bin/bash')



    def terminate_ros_node(self):

        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()

        print("list cmd: {}".format(list_cmd))
        print("list output: {}".format(list_output))
        print("retcode: {}".format(retcode))

        assert retcode == 0, "List command returned %d" % retcode
        for msg in list_output.split("\n"):

            print("msg: {}".format(msg))

            # if (msg.startswith("/" + cls.node_name)):
            if (msg.startswith("/" + self.rosbag_script_node_name)):
                os.system("rosnode kill " + msg)



if __name__ == '__main__':

    rosbag_script = sys.argv[1]
    rosbag_folder = sys.argv[2]
    rosbag_start_recording = False

    try:
        rosbag_start_recording = bool(sys.argv[3])
    except Exception:
        pass

    # Go to class functions that do all the heavy lifting. Do error checking.
    # try:
    rosbag_record = RosbagRecord(rosbag_script, rosbag_folder, rosbag_start_recording)
    # except rospy.ROSInterruptException as e:
    #     print("Exception running rosbag_record node: {}".format(e))
    #     pass