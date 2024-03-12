#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped
from nav_msgs.msg import Path, Odometry

import numpy as np
from tf.transformations import euler_from_quaternion
from tf_conversions import Quaternion

class Inspection():
    def __init__(self):
        # Topic Defintions
        self.TOPIC_CMD_VEL = "/cmd_vel"
        self.TOPIC_FRONT_SCAN = "/front/scan"
        self.TOPIC_LOCAL_FOOTPRINT = "/move_base/local_costmap/footprint"
        self.TOPIC_GLOBAL_PLAN = "/move_base/TrajectoryPlannerROS/global_plan"
        self.TOPIC_LOCAL_PLAN = "/move_base/TrajectoryPlannerROS/local_plan"
        self.TOPIC_ODOM = "/odometry/filtered"
        
        # Object to store
        self.scan = LaserScan()
        self.cmd_vel = Twist()
        self.global_plan = Path()
        self.local_plan = Path()
        self.odometry = Odometry()
        self.footprint = PolygonStamped()

        # Subscribe        
        self.sub_front_scan = rospy.Subscriber(self.TOPIC_FRONT_SCAN, LaserScan, self.callback_front_scan)
        self.sub_odometry = rospy.Subscriber(self.TOPIC_ODOM, Odometry, self.callback_odometry)
        self.sub_global_plan = rospy.Subscriber(self.TOPIC_GLOBAL_PLAN, Path, self.callback_global_plan)
        self.sub_local_plan = rospy.Subscriber(self.TOPIC_LOCAL_PLAN, Path, self.callback_local_plan)
        self.sub_footprint = rospy.Subscriber(self.TOPIC_LOCAL_FOOTPRINT, PolygonStamped, self.callback_footprint)
        self.sub_cmd_vel = rospy.Subscriber(self.TOPIC_CMD_VEL, Twist, self.callback_cmd_vel)
        self.publish_cmd_vel = rospy.Publisher(self.TOPIC_CMD_VEL, Twist, queue_size= 10)
        pass
    
    def callback_front_scan(self,data):
        self.scan = data
        if 0:
            print("Scan points: ", len(data.ranges), "From Max: ", data.range_max, "| Min: ", round(data.range_min,2))
            print("Angle from: ", np.degrees(data.angle_min).round(2), " to: ", np.degrees(data.angle_max).round(2), " increment: ", np.degrees(data.angle_increment).round(3))

        pass
    def callback_global_plan(self,data):
        self.global_plan = data
        if 1: 
            print("Global Path points ", len(data.poses))
            print(data.poses[3])
            print("Local Path points ", len(self.local_plan.poses))
    def callback_local_plan(self,data):
        self.local_plan = data
    
    def callback_odometry(self,data):
        if 0:
            self.odometry = data
            print("==========================")
            print("----------------------- pose.position")
            print(data.pose.pose.position)
            print("----------------------- pose.orientation")
            print(data.pose.pose.orientation)
            q = Quaternion()
            q.x = data.pose.pose.orientation.x
            q.y = data.pose.pose.orientation.y
            q.z = data.pose.pose.orientation.z
            q.w = data.pose.pose.orientation.w
            print("----------------------- pose.heading")
            heading_rad = np.array(euler_from_quaternion([q.x, q.y, q.z,q.w])[2])
            heading_deg = np.degrees(heading_rad)
            print("Rad: " + str(heading_rad.round(3)))
            print("Degree: " + str( heading_deg.round(3)))

            print("----------------------- twist.linear")
            print(data.twist.twist.linear)
            print("----------------------- twist.angular")
            print(data.twist.twist.angular)
        pass
    
    
    def callback_footprint(self,data):
        self.footprint = data
        if 0: 
            points_array = []
            for point in data.polygon.points:
                points_array.append([point.x, point.y,point.z])
            np_array = np.array(points_array)
            print("Number of points on the Polygon: ", len(data.polygon.points))
            print("Points: ", np.round(np_array,3))
        pass
    def callback_cmd_vel(self,data):
        if 0:
            print("Linear: ", data.linear, "; Angular: ", data.angular)
        pass



if __name__ == "__main__":
    rospy.init_node('inspection_node')
    rospy.loginfo("Inspection Node Started")
    inspect = Inspection()
    rospy.spin()
    