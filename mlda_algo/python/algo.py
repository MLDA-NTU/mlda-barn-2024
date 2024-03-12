#!/usr/bin/python
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped,Pose
from nav_msgs.msg import Path, Odometry

import numpy as np
from tf.transformations import euler_from_quaternion
from tf_conversions import Quaternion


class SimpleControl:

    def __init__(self):
        # Attribute
        self.odom = Odometry()
        self.error = 0
        self.prev_error = 0
        self.delta_error = 0
        self.cummulative_error = 0
        
        # Topic Title
        self.TOPIC_CMD_VEL = "/cmd_vel"
        self.TOPIC_GLOBAL_PLAN = "/move_base/TrajectoryPlannerROS/global_plan"
        self.TOPIC_ODOM = "/odometry/filtered"
        
        # Subscriber
        self.sub_global_plan = rospy.Subscriber(self.TOPIC_GLOBAL_PLAN, Path, self.callback_global_plan)
        self.sub_odom = rospy.Subscriber(self.TOPIC_ODOM, Odometry, self.callback_odom)
        self.pub_cmd_vel = rospy.Publisher(self.TOPIC_CMD_VEL, Twist, queue_size=10)

    def callback_odom(self,data):
        self.odom = data
    
    def callback_global_plan(self, data):
        if len(data.poses) < 11:
            IDX = 0
            v = 0
            omega = 0
        else: 
            IDX = 10
            target_pose = data.poses[IDX]
            print("===== TARGET POSE =====")
            print(target_pose)
            target = [0]*2
            target[0] = target_pose.pose.position.x
            target[1] = target_pose.pose.position.y
            
            current = [0]*2
            current_pose = self.odom.pose
            current[0] = current_pose.pose.position.x
            current[1] = current_pose.pose.position.y
            
            target_heading = np.degrees(np.arctan2(target[1] - current[1], target[0] - current[0]))
            
            # target_heading = self.heading_from_pose(target_pose.pose)
            current_heading = self.heading_from_pose(self.odom.pose.pose)
            
            print("Target {}, Current {}".format(target_heading, current_heading))
            K_p = 0.1
            K_d = 0
            K_i = 0
            
            v = 0
            self.error = (target_heading- current_heading)
            self.cummulative_error += self.error
            self.delta_error = self.error - self.prev_error
            omega = K_p * self.error + K_i * self.cummulative_error + K_d * self.delta_error
            print("Raw {}".format(omega))
            omega = max(-1, min(1, omega))
            omega = 0 if abs(self.error) < 2 else omega
            self.prev_error = self.error
            
            if abs(self.error) < 2:
                v = 0.5
            else:
                v = 0
        
        vel_cmd = Twist()
        vel_cmd.linear.x = v
        vel_cmd.angular.z = omega
        print("Error {}. Cummulative Error {}".format(self.error, self.cummulative_error))
        print("v: {} and omega {}".format(v, omega))
        self.pub_cmd_vel.publish(vel_cmd)
        
        
    def heading_from_pose(self, q):
        a = [0]*4
        a[0] = q.orientation.x
        a[1] = q.orientation.y
        a[2] = q.orientation.z
        a[3] = q.orientation.w
        heading_degree = np.degrees(np.array(euler_from_quaternion([a[0],a[1],a[2],a[3]])[2]))
        return heading_degree

if __name__ == "__main__":
    rospy.init_node("simple_control")
    rospy.loginfo("Simple Control Started")
    control = SimpleControl()
    rospy.spin()
    # while not rospy.is_shutdown():
        