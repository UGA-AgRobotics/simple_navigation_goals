#!/usr/bin/env python

# import roslib; roslib.load_manifest('robot_red')
import rospy
import actionlib
import utm
import sys

#move_base_msgs
# from move_base_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from sensor_msgs.msg import NavSatFix
from actionlib.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from simple_navigation_goals.srv import *


TestFlagLat = 31.4753776881
TestFlagLon = -83.5289577629
TestFlagUTM = utm.from_latlon(TestFlagLat, TestFlagLon)


def convert_to_utm(lat, lon):
    try:
        return utm.from_latlon(lat, lon)
    except e as Exception:
        print("Error converting lat/lon to utm: {}".format(e))
        return None


def call_jackal_pos_service():
    # Get current GPS fix from Jackal's position:
    print("Requestion Jackal's current position via get_jackal_pos service..")
    rospy.wait_for_service('get_jackal_pos')
    get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
    current_jackal_position = get_jackal_pos()
    print("Jackal's current position: {}".format(current_jackal_position))

    current_jackal_position_utm = convert_to_utm(current_jackal_position.jackal_fix.latitude, current_jackal_position.jackal_fix.longitude)

    print("Jackal's current position in UTM: {}".format(current_jackal_position_utm))

    return current_jackal_position_utm


# def convert_


def simple_move(x=0.1, y=0.1, w=0.01, frame='base_link'):

    rospy.init_node('simple_navigation_goals')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    print("Initiating simple_move action..")

    current_jackal_position_utm = call_jackal_pos_service()  # get Jackal's current position in UTM

    #create goal
    goal = MoveBaseGoal()

    print("Creating goal...")

    x_diff = round(abs(TestFlagUTM[0] - current_jackal_position_utm[0]))  # magnitude in X direction
    y_diff = round(abs(TestFlagUTM[1] - current_jackal_position_utm[1]))  # magnitude in Y direction


    # #set goal
    # goal.target_pose.pose.position.x = x_diff
    # goal.target_pose.pose.position.y = y_diff
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # goal.target_pose.pose.orientation.x = x
    # goal.target_pose.pose.orientation.y = y
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.header.frame_id = frame  # testing 'navsat_link' as frame_id when outside..
    goal.target_pose.header.stamp = rospy.Time.now()

    print("Setting goal with the following x,y scalars: {}m, {}m".format(x_diff, y_diff))
    # print("Setting goal with the following x,y,w,frame {}, {}, {}, {}".format(x, y, w, frame))

    # print("Goal set.")
    print("Waiting for server...")

    # #start listner
    sac.wait_for_server()

    print("Sending goal...")

    # #send goal
    sac.send_goal(goal)

    print("Waiting for result...")

    # #finish
    sac.wait_for_result()

    # #print result
    print("Result: {}".format(sac.get_result()))

    print("goal.target_pose.pose.position: {}".format(dir(goal.target_pose.pose.position)))
    print("goal.target_pose.pose.orientation: {}".format(dir(goal.target_pose.pose.orientation)))

    print("position _type: {}".format(goal.target_pose.pose.position._type))
    print("orientation _type: {}".format(goal.target_pose.pose.orientation._type))

    # # rospy.spin()




def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 0.5
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   




# From http://www.hotblackrobotics.com/en/blog/2018/01/29/seq-goals-py/
class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaw_euler_angles_seq = rospy.get_param('move_base_seq/yea_seq')
        quat_seq = list()

        self.pose_seq = list()
        self.goal_cnt = 0

        for yaw_angle in yaw_euler_angles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yaw_angle * math.pi / 180, axes='sxyz'))))

        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n - 3]))
            n += 1

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        rospy.loginfo("Connected to move base server...")
        rospy.loginfo("Starting goals achievements...")

        self.movebase_client()


    def active_cb(self):
        rospy.loginfo("Goal pose {} is now being process by the Action Server...".format(self.goal_cnt + 1))
   

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose {} received".format(self.goal_cnt + 1))


    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose {} received a cancel request after it started executing, completed execution!".format(self.goal_cnt))

        if status == 3:
            rospy.loginfo("Goal pose {} reached".format(self.goal_cnt))
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"  #?????
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose {} to Action Server".format(self.goal_cnt + 1))
                rospy.loginfo("{}".format(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose {} was aborted by the Action Server".format(self.goal_cnt))
            rospy.signal_shutdown("Goal pose {} aborted, shutting down!".format(self.goal_cnt))
            return

        if status == 5:
            rospy.loginfo("Goal pose {} has been rejected by the Action Server".format(self.goal_cnt))
            rospy.signal_shutdown("Goal pose {} rejected, shutting down!".format(self.goal_cnt))
            return

        if status == 8:
            rospy.loginfo("Goal pose {} received a cancel request before it started executing, successfully canceled!".format(self.goal_cnt))


    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  #??????
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose {} to Action Server".format(self.goal_cnt + 1))
        rospy.loginfo("{}".format(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()


if __name__ == '__main__':
    
    # 1. Run simple_move() function
    # try:
    #     # X, Y, W, and frame_id as sys args:
    #     _x, _y, _w, _frame = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    #     print ("X: {}, Y: {}, W: {}, Frame: {}".format(_x, _y, _w, _frame))
    #     simple_move(_x, _y, _w, _frame)
    # # except rospy.ROSInterruptException:
    # except Exception:
    #     print("no args for jackal_pos_client, using defaults")
    #     simple_move()



    # 2. Run movebase_client() from http://www.hotblackrobotics.com/en/blog/2018/01/29/action-client-py/#1-the-actionlib-library
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



    # 3. Run MoveBaseSeq class from http://www.hotblackrobotics.com/en/blog/2018/01/29/seq-goals-py/
    # try:
    #     MoveBaseSeq()
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation finished.")