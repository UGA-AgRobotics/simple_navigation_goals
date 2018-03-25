#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
# from transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, degrees
import utm
import PyKDL
from simple_navigation_goals.srv import *



def turn_jackal(goal_angle=0):

    # Get current IMU data from get_jackal_rot service
    rospy.wait_for_service('get_jackal_rot')
    get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
    current_jackal_rotation = get_jackal_rot()

    quat = current_jackal_rotation.jackal_rot.orientation  # get quaternion from jackal imu
    curr_angle = degrees(quat_to_angle(quat))  # convert quaternion to angle
    delta_angle = curr_angle - goal_angle  # determine angle to turn

    print "Current angle: {}, Goal angle: {}, Delta angle: {}".format(curr_angle, goal_angle, delta_angle)

    # Stop the robot before rotating
    move_cmd = Twist()
    self.cmd_vel.publish(move_cmd)
    rospy.sleep(1.0)
    
    # Set the movement command to a rotation
    move_cmd.angular.z = angular_speed
    
    # Track the last angle measured
    last_angle = rotation
    

    move_cmd = Twist()
    self.cmd_vel.publish(move_cmd)
    rospy.sleep(1.0)
        
    # Stop the robot when we are done
    self.cmd_vel.publish(Twist())




def quat_to_angle(quat):
    """
    Converts quaternion to angle.
    """
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]



if __name__ == '__main__':
    try:
        # SingleGoalNav()
        turn_jackal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")