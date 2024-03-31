#!/usr/bin/python3
import casadi
import rospy
import numpy as np
import mpc_algo_right_left as mpc_algo
import math
import time

from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped,PolygonStamped, Quaternion
from visualization_msgs.msg import Marker

# import tf

class ROSNode():
    def __init__(self):
        self.TOPIC_VEL = "/cmd_vel"
        self.TOPIC_GLOBAL_PLAN = "/move_base/TrajectoryPlannerROS/global_plan"
        self.TOPIC_LOCAL_PLAN = "/move_base/TrajectoryPlannerROS/local_plan"
        self.TOPIC_ODOM = "/odometry/filtered"
        self.TOPIC_MPC_PLAN = "/mpc_plan"
        self.TOPIC_PATH = "/path"
        
        self.pub_vel = rospy.Publisher(self.TOPIC_VEL, Twist, queue_size=1, latch=True)
        self.pub_mpc  = rospy.Publisher(self.TOPIC_MPC_PLAN, Path, queue_size=1)
        self.path  = rospy.Publisher(self.TOPIC_PATH, Path, queue_size=1)
        
        self.sub_odometry = rospy.Subscriber(self.TOPIC_ODOM, Odometry, self.callback_odom)
        self.sub_global_plan = rospy.Subscriber(self.TOPIC_GLOBAL_PLAN, Path, self.callback_global_plan)
        self.sub_local_plan = rospy.Subscriber(self.TOPIC_LOCAL_PLAN, Path, self.callback_local_plan)

        self.cmd_vel = Twist()
        self.odometry = Odometry()
        self.global_plan = Path()
        self.local_plan = Path()
        
        self.rate = 10
        self.N = 20
        self.mpc = mpc_algo.NMPC(freq=self.rate, N=self.N)
        self.v_opt = 0 
        self.w_opt = 0
        

        self.x_ref = []
        self.y_ref = []
        self.theta_ref = []
        
        self.deviation_threshold = 0.3
        self.follow_threshold = 15
        self.follow = 0
    
    def callback_odom(self,data):
        self.odometry = data
        yaw = self.quaternion_to_yaw(data.pose.pose.orientation)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        v = data.twist.twist.linear.x
        w = data.twist.twist.angular.z
        vr = v + w*self.mpc.L/2
        vl = v - w*self.mpc.L/2
        self.X0 = [x,y,yaw,vr,vl]

    def callback_global_plan(self,data):
        # self.global_plan = data
        # self.x_ref = [pose.pose.position.x for pose in self.global_plan.poses[::]]
        # self.y_ref = [pose.pose.position.y for pose in self.global_plan.poses[::]]
        # self.theta_ref = []
        # for i in range(len(self.x_ref)-1):
        #     theta = math.atan2((self.y_ref[i+1] - self.y_ref[i]),(self.x_ref[i+1] - self.x_ref[i]))
        #     self.theta_ref.append(theta)
        #     if i == 0:
        #         self.theta_ref.append(theta)
        # print("Global poses: ",len(data.poses))
        # print("X_ref ",len(self.x_ref), " : ", self.x_ref)
        # print("Y_ref ",len(self.y_ref), " : ", self.y_ref)
        # print("Theta_ref ", len(self.theta_ref), " : ", self.theta_ref)
        
        # Update true global plan
        # self.true_global_plan = self.global_plan
        # if self.true_global_plan == Path():
        #     self.true_global_plan = self.global_plan
        #     print("Initialized path")
        # else:
        #     x_ref, y_ref, theta_ref = self.get_ref_from_path(self.true_global_plan)
        #     deviation = 0
            
        #     # find the portion of path that aligns
        #     align_index = -1
        #     prev_dist = 10e9
        #     for i in range(self.N):
        #         dist = math.sqrt((x_ref[i+align_index]-self.x_ref[i])**2 + (y_ref[i+align_index]-self.y_ref[i])**2)
        #         if prev_dist < dist: break
        #         else: align_index = i
        #         prev_dist = dist
                
        #     for i in range(self.N-align_index):
        #         dist = math.sqrt((x_ref[i+align_index]-self.x_ref[i])**2 + (y_ref[i+align_index]-self.y_ref[i])**2)
        #         deviation += dist
                
        #     mean_deviation = deviation/(self.N - align_index)
        #     if mean_deviation > self.deviation_threshold:
        #         self.true_global_plan = self.global_plan
        #         print("Following new path")
        #     else:
        #         print("Following original path")
        
        x_ref, y_ref, theta_ref = self.get_ref_from_path(data)
        
        
        if self.x_ref == []:
            self.x_ref = x_ref
            self.y_ref = y_ref
            self.theta_ref = theta_ref
            self.global_plan = data
            print("Initialized")
            
        else:
            deviation = 0
            #  find the portion of path that aligns
            align_index = 0
            prev_dist = 10e9
            for i in range(min(len(x_ref), len(self.x_ref))):
                dist = math.sqrt((self.x_ref[0]-x_ref[i])**2 + (self.y_ref[0]-y_ref[i])**2)
                if prev_dist < dist: break
                align_index = i
                prev_dist = dist
            # print("align_index: ", align_index)
            # calculate mean deviation
            for i in range(min(len(x_ref), len(self.x_ref))-align_index):
                dist = math.sqrt((self.x_ref[i+align_index]-x_ref[i])**2 + (self.y_ref[i+align_index]-y_ref[i])**2)
                deviation += dist
            mean_deviation = deviation/(min(len(x_ref), len(self.x_ref))-align_index)
            print("Mean deviation: ", mean_deviation)

            if mean_deviation > self.deviation_threshold or self.follow > self.follow_threshold:
                self.x_ref = x_ref
                self.y_ref = y_ref
                self.theta_ref = theta_ref
                self.global_plan = data
                self.follow = 0
                print("Following new path")
            else:
                self.follow += 1
                print("Following original path")
                
        
            
        
        
    def callback_local_plan(self, data):
        self.local_plan = data
        # print("Local")

    def quaternion_to_yaw(self, orientation):
    # Convert quaternion orientation data to yaw angle of robot
        q0 = orientation.x
        q1 = orientation.y
        q2 = orientation.z
        q3 = orientation.w
        theta = math.atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2))
        # print("Yaw: ", theta)
        return theta
    
    def publish_trajectory(self, mpc_x_traj, mpc_y_traj):
        mpc_traj_msg = Path()
        mpc_traj_msg.header.stamp = rospy.Time.now()
        mpc_traj_msg.header.frame_id = "odom"
        for i in range(mpc_x_traj.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x = mpc_x_traj[i]
            pose.pose.position.y = mpc_y_traj[i]
            pose.pose.orientation = Quaternion(0,0,0,1)
            mpc_traj_msg.poses.append(pose)
            
        self.pub_mpc.publish(mpc_traj_msg)
        
    def publish_path_trajectory(self, mpc_x_traj, mpc_y_traj):
        mpc_traj_msg = Path()
        mpc_traj_msg.header.stamp = rospy.Time.now()
        mpc_traj_msg.header.frame_id = "odom"
        for i in range(mpc_x_traj.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x = mpc_x_traj[i]
            pose.pose.position.y = mpc_y_traj[i]
            pose.pose.orientation = Quaternion(0,0,0,1)
            mpc_traj_msg.poses.append(pose)
            
        self.path.publish(mpc_traj_msg)

    def publish_velocity(self, v_opt, w_opt):
        vel = Twist()
        vel.linear.x = v_opt
        vel.angular.z = w_opt
        self.pub_vel.publish(vel)
        
    def get_ref_from_path(self, path):
        x_ref = [pose.pose.position.x for pose in path.poses[::2]]
        y_ref = [pose.pose.position.y for pose in path.poses[::2]]
        theta_ref = []
        for i in range(len(x_ref)-1):
            theta = math.atan2((y_ref[i+1] - y_ref[i]),(x_ref[i+1] - x_ref[i]))
            theta_ref.append(theta)
            if i == 0:
                theta_ref.append(theta)
        return x_ref, y_ref, theta_ref
        
    def run(self):
        # take ref from true global plan instead of current global plan
        # self.x_ref, self.y_ref, self.theta_ref = self.get_ref_from_path(self.true_global_plan)
        
        # try:
        if len(self.x_ref) > self.mpc.N:
            
            # Setup the MPC
            #TODO: Do this
            self.mpc.setup(self.rate)
            # solve
            self.v_opt, self.w_opt, solve_time = self.mpc.solve(self.x_ref, self.y_ref, self.theta_ref, self.X0) # Return the optimization variables
            # Control and take only the first step 
            
            rospy.loginfo("Solve time: " + str(solve_time))
            
            self.publish_velocity(self.v_opt, self.w_opt)

            
            # Get from the MPC results
            mpc_x_traj = self.mpc.opt_states[0::self.mpc.n]
            mpc_y_traj = self.mpc.opt_states[1::self.mpc.n]
            # print(type(mpc_x_traj), mpc_x_traj.shape)
            self.publish_trajectory(mpc_x_traj, mpc_y_traj)
            self.publish_path_trajectory(np.array(self.x_ref), np.array(self.y_ref))
            self.x_ref = self.x_ref[1:]
            self.y_ref = self.y_ref[1:]
        else:
            print("Stopped", len(self.x_ref))
            self.publish_velocity(0,0)
        # except Exception as e:
        #     rospy.logerr(e)


def start_traj():
    start_traj_publisher = rospy.Publisher("/start_traj", Path, queue_size=1, latch=True)
    traj = rospy.wait_for_message("/move_base/TrajectoryPlannerROS/global_plan", Path,timeout=1) # GEt msg 1 time
    # mpc_traj_msg = Path()
    # mpc_traj_msg.header.stamp = rospy.Time.now()
    # mpc_traj_msg.header.frame_id = "odom"
    # for i in range(traj.shape[0]):
    #     pose = PoseStamped()
    #     pose.pose.position.x = traj[i]
    #     pose.pose.position.y = traj[i]
    #     pose.pose.orientation = Quaternion(0,0,0,1)
    #     mpc_traj_msg.poses.append(pose)
    start_traj_publisher.publish(traj)
    pass
if __name__ =="__main__":
    rospy.init_node("nmpc")
    rospy.loginfo("Non-Linear MPC Node running")
    node = ROSNode()
    pause = rospy.Rate(node.rate)
    time.sleep(1)
    start_traj()
    while not rospy.is_shutdown():
        node.run()
        pause.sleep()