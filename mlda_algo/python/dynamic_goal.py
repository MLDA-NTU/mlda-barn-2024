#!/usr/bin/python

INIT_POSITION = [11, 0, 3.14]  # in world frame
GOAL_POSITION = [-8, 0]  # relative to the initial position

import rospy
import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction



if __name__ == "__main__":
    rospy.init_node("set_goal")
    rospy.loginfo("SET GOAL")
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom'
    mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
    mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
    mb_goal.target_pose.pose.position.z = 0
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
    nav_as.wait_for_server()
    nav_as.send_goal(mb_goal)