#!/usr/bin/env python

""" nav_square.py - Version 0.1 2012-03-24

    A basic demo of the using odometry data to move the robot
    along a square trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from simple_navigation_goals.srv import *
import tf
# from transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, degrees
import utm
import PyKDL



# TODO: Move this somewhere that makes sense
TestFlagLat = 31.4753776881
TestFlagLon = -83.5289577629
TestFlagUTM = utm.from_latlon(TestFlagLat, TestFlagLon)

global_current_orientation = None



class SingleGoalNav():
    """
    Testing Jackal navigation to a single goal/flag. Determines
    X and Y distance to travel using its GPS location and flag's 
    location, both converted to UTM.
    """

    def __init__(self):
        # Give the node a name
        rospy.init_node('single_goal_nav', anonymous=False)
        
        # Set rospy to exectute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        rate = 20
        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)
        
        # Set the parameters for the target square
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = rospy.get_param("~goal_angle", radians(90))    # degrees converted to radians
        linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
        angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
        angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
         
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
                

        position = Point()  # initialize the position variable as a Point type
        move_cmd = Twist()  # initialize movement comment

        move_cmd.linear.x = linear_speed  # set movement command to forward motion

        (position, rotation) = self.get_odom()  # get starting position values

        x_start, y_start = position.x, position.y  # set start positions


        """
        Scenario 1 -- Determine angle to turn to face flag, then travel straight to the flag.

        Order of events:
            1. Determine Jackal's current position and orientation.
            2. Calculate angle for Jackal to turn to travel toward goal flag.
            3. Turn Jackal toward the goal flag, then pause for a second.
            4. Calculate distance between Jackal's position and goal flag.
            5. Drive Jackal to flag using distance calculated in 4.
        """

        track1 = [[1, 0], [1, 1]]  # creating coords for 1m forward, 1m left track

        # while not rospy.is_shutdown():
            # _jackal_pos = self.call_jackal_pos_service()  # get jackal's current position and orientation
            # _angle = self.quat_to_angle()

        (position, rotation) = self.get_odom()

        print("Jackal's current position: {}".format(position))  # get_odom() seems to already do the above 2 commented out steps
        print("Jackal's current orientation: {}".format(rotation))

        rospy.sleep(1)


        # 1. Recreate turn command from yesterday's jackal_rot_server.py testing: manually telling
        # the jackal rotation server to turn the jackal 45 degrees..
        self.call_jackal_rot_service(45)  # tell jackal to turn 45 degrees


        return



        # rospy.spin()  # temp loop for determining jackal's position/orientation base on reference frame, etc.
            
        # Stop the robot before rotating
        # move_cmd = Twist()
        # self.cmd_vel.publish(move_cmd)
        # rospy.sleep(1.0)
        
        # Set the movement command to a rotation
        # move_cmd.angular.z = angular_speed
        
        # Track the last angle measured
        # last_angle = rotation
        

        # move_cmd = Twist()
        # self.cmd_vel.publish(move_cmd)
        # rospy.sleep(1.0)
            
        # Stop the robot when we are done
        # self.cmd_vel.publish(Twist())


    def turn_jackal(goal_angle=0):
        # Get current IMU data from get_jackal_rot service
        rospy.wait_for_service('get_jackal_rot')
        get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
        current_jackal_rotation = get_jackal_rot()

        quat = current_jackal_rotation.jackal_rot.orientation  # get quaternion from jackal imu
        curr_angle = degrees(quat_to_angle(quat))  # convert quaternion to angle
        delta_angle = curr_angle - goal_angle  # determine angle to turn

        print "Current angle: {}, Goal angle: {}, Delta angle: {}".format(curr_angle, goal_angle, delta_angle)


    def execute_movement(self, position, rotation, goal_distance):
        """
        Function for driving straight.
        """

        distance = 0  # keep track of distance traveled

        # Initiate movement loop:
        while distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            self.r.sleep()
    
            # Get the current position
            (position, rotation) = self.get_odom()
            
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

        return True


    def execute_turn(self, position, rotation, last_angle, angular_tolerance, goal_angle):
        """
        Function for executing a turn in the odom frame.
        """

        turn_angle = 0  # keep track of turning angle

        # Begin the rotation
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            self.r.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_angle = self.normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation

        return True


    def determine_turn_angle(self):
        """
        Calculates angle to turn from Jackal's current orientation
        to the goal flag's position.
        """



    def get_odom(self):
        """
        Get the current transform between the odom and base frames
        """
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))


    def quat_to_angle(self, quat):
        """
        Converts quaternion to angle.
        """
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]


    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
        

    def shutdown(self):
        """
        Always stop the robot when shutting down the node
        """
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def convert_to_utm(self, lat, lon):
        """
        Convert lat/lon to utm.
        """
        try:
            return utm.from_latlon(lat, lon)
        except e as Exception:
            print("Error converting lat/lon to utm: {}".format(e))
            return None


    def call_jackal_pos_service(self):
        """
        Get current GPS fix from Jackal's position
        """
        rospy.wait_for_service('get_jackal_pos')
        get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
        return get_jackal_pos()


    def call_jackal_rot_service(self, angle):
        """
        Get current IMU position and orientation from Jackal.
        Inputs:
            angle - angle to turn in radians
        """
        rospy.wait_for_service('get_jackal_rot')
        get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
        return get_jackal_rot(angle)



if __name__ == '__main__':
    try:
        # SingleGoalNav().call_jackal_pos_service()
        SingleGoalNav()
        # turn_jackal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")