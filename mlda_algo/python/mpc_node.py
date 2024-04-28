#!/usr/bin/python3

import casadi
import rospy
import numpy as np
import mpc_algo_right_left as mpc_algo
import math
import time


from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg


# import tf


class ROSNode:
    def __init__(self):
        self.TOPIC_VEL = "/cmd_vel"
        self.TOPIC_GLOBAL_PLAN = "/move_base/TrajectoryPlannerROS/global_plan"
        self.TOPIC_LOCAL_PLAN = "/move_base/TrajectoryPlannerROS/local_plan"
        self.TOPIC_ODOM = "/odometry/filtered"
        self.TOPIC_MPC_PLAN = "/mpc_plan"
        self.TOPIC_CLOUD = "/front/odom/cloud"
        self.TOPIC_MAP_CLOUD = "/map/cloud"
        self.TOPIC_MPC_INIT = "/mpc_init"

        self.pub_vel = rospy.Publisher(self.TOPIC_VEL, Twist, queue_size=1, latch=True)
        self.pub_mpc = rospy.Publisher(self.TOPIC_MPC_PLAN, Path, queue_size=1)
        self.pub_mpc_init = rospy.Publisher(self.TOPIC_MPC_INIT, Marker, queue_size=1)

        self.sub_odometry = rospy.Subscriber(
            self.TOPIC_ODOM, Odometry, self.callback_odom
        )
        self.sub_global_plan = rospy.Subscriber(
            self.TOPIC_GLOBAL_PLAN, Path, self.callback_global_plan
        )
        self.sub_local_plan = rospy.Subscriber(
            self.TOPIC_LOCAL_PLAN, Path, self.callback_local_plan
        )
        self.sub_cloud = rospy.Subscriber(
            self.TOPIC_CLOUD, PointCloud2, self.callback_cloud
        )
        self.sub_map_cloud = rospy.Subscriber(
            self.TOPIC_MAP_CLOUD, PointCloud2, self.callback_map_cloud
        )

        self.projector = lg.LaserProjection()
        self.cmd_vel = Twist()
        self.odometry = Odometry()
        self.global_plan = Path()
        self.local_plan = Path()

        self.rate = 10
        self.N = 20
        print(" N: ", self.N)
        self.mpc = mpc_algo.NMPC(freq=self.rate, N=self.N)
        self.v_opt = 0
        self.w_opt = 0

        self.x_ref = []
        self.y_ref = []
        self.obs_x = []
        self.obs_y = []
        self.og_x_ref = []
        self.og_y_ref = []
        self.theta_ref = []
        self.count = 0

    def callback_cloud(self, data):
        point_generator = pc2.read_points(data)
        self.obs_x = []
        self.obs_y = []
        for point in point_generator:
            self.obs_x.append(point[0])
            self.obs_y.append(point[1])

    def callback_map_cloud(self, data):
        point_generator = pc2.read_points(data)
        self.map_x = []
        self.map_y = []
        for point in point_generator:
            self.map_x.append(point[0])
            self.map_y.append(point[1])

    def callback_odom(self, data):
        self.odometry = data
        yaw = self.quaternion_to_yaw(data.pose.pose.orientation)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        v = data.twist.twist.linear.x
        w = data.twist.twist.angular.z
        vr = v + w * self.mpc.L / 2
        vl = v - w * self.mpc.L / 2
        self.X0 = [x, y, yaw, vr, vl]

    def callback_global_plan(self, data):
        self.global_plan = data
        if 1:
            if len(data.poses) <= 2 * (self.mpc.N + 5):
                self.og_x_ref = [
                    pose.pose.position.x for pose in self.global_plan.poses[::1]
                ]
                self.og_y_ref = [
                    pose.pose.position.y for pose in self.global_plan.poses[::1]
                ]
            else:
                self.og_x_ref = [
                    pose.pose.position.x for pose in self.global_plan.poses[::2]
                ]
                self.og_y_ref = [
                    pose.pose.position.y for pose in self.global_plan.poses[::2]
                ]
            self.theta_ref = []
            center_heading = self.X0[2]
            for i in range(len(self.og_x_ref) - 1):
                theta = math.atan2(
                    (self.og_y_ref[i + 1] - self.og_y_ref[i]),
                    (self.og_x_ref[i + 1] - self.og_x_ref[i]),
                )
                theta_preprocessed = self.heading_preprocess_radian(
                    center_heading, theta
                )
                self.theta_ref.append(theta_preprocessed)
                if i == 0:
                    self.theta_ref.append(theta_preprocessed)
                center_heading = theta_preprocessed

    def callback_local_plan(self, data):
        self.local_plan = data
        # print("Local")

    def heading_preprocess_radian(self, center, target):
        min = center - np.pi
        max = center + np.pi
        if target < min:
            while target < min:
                # print('Processed')
                target += 2 * np.pi
        elif target > max:
            while target > max:
                # print('Processed')
                target -= 2 * np.pi
        return target

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion orientation data to yaw angle of robot
        q0 = orientation.x
        q1 = orientation.y
        q2 = orientation.z
        q3 = orientation.w
        theta = math.atan2(2.0 * (q2 * q3 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
        # print("Yaw: ", theta)
        return theta

    def publish_trajectory(self, publisher, mpc_x_traj, mpc_y_traj):
        mpc_traj_msg = Path()
        mpc_traj_msg.header.stamp = rospy.Time.now()
        mpc_traj_msg.header.frame_id = "odom"
        for i in range(mpc_x_traj.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x = mpc_x_traj[i]
            pose.pose.position.y = mpc_y_traj[i]
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            mpc_traj_msg.poses.append(pose)

        publisher.publish(mpc_traj_msg)

    def publish_velocity(self, v_opt, w_opt):
        vel = Twist()
        vel.linear.x = v_opt
        vel.angular.z = w_opt
        self.pub_vel.publish(vel)

    def run(self):
        # try:
        # Clear up to the closest point
        try:
            k = 0
            dist = [1] * len(self.og_x_ref)
            min_dist = 1
            for i in range(len(self.og_x_ref)):
                dist[i] = math.sqrt(
                    (self.og_x_ref[i] - self.odometry.pose.pose.position.x) ** 2
                    + (self.og_y_ref[i] - self.odometry.pose.pose.position.y) ** 2
                )
                if dist[i] <= min_dist:
                    k = i
                    min_dist = dist[i]

            self.x_ref = self.og_x_ref[k:]
            self.y_ref = self.og_y_ref[k:]
            # print("Before loop: ", len(self.x_ref))

            all_obs_x = self.obs_x + self.map_x
            all_obs_y = self.obs_y + self.map_y
            # print("Number of Obstacles: ", len(self.obs_x), len(self.map_x), len(all_obs_x))
            if len(self.x_ref) > self.mpc.N:

                # Setup the MPC
                # print("Before solve: ", len(self.x_ref))
                if len(all_obs_x) == 0:

                    self.v_opt, self.w_opt = self.mpc.solve(
                        self.x_ref, self.y_ref, self.theta_ref, self.X0
                    )  # Return the optimization variables
                else:
                    self.v_opt, self.w_opt = self.mpc.solve_obs(
                        self.x_ref,
                        self.y_ref,
                        self.theta_ref,
                        all_obs_x,
                        all_obs_y,
                        self.X0,
                    )  # Return the optimization variables

                # Control and take only the first step
                self.publish_velocity(self.v_opt, self.w_opt)

                # Get from the MPC results
                mpc_x_traj = self.mpc.opt_states[0 :: self.mpc.n]
                mpc_y_traj = self.mpc.opt_states[1 :: self.mpc.n]
                mpc_init_x_traj = self.mpc.init_guess[0 :: self.mpc.n]
                mpc_init_y_traj = self.mpc.init_guess[1 :: self.mpc.n]
                # print(type(mpc_x_traj), mpc_x_traj.shape)
                self.publish_trajectory(self.pub_mpc, mpc_x_traj, mpc_y_traj)
                # self.publish_trajectory(self.pub_mpc_init, mpc_init_x_traj, mpc_init_y_traj)

                # self.x_ref = self.x_ref[1:]
                # self.y_ref = self.y_ref[1:]

                # print("K: ", k)
                # print(dist)

            else:
                print("Stopped", len(self.x_ref))
                self.publish_velocity(0, 0)
        except Exception as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node("nmpc")
    rospy.loginfo("Non-Linear MPC Node running")
    node = ROSNode()
    pause = rospy.Rate(10)
    time.sleep(1)
    while not rospy.is_shutdown():
        node.run()
        pause.sleep()
