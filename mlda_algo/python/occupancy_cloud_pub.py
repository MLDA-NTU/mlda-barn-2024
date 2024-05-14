#!/usr/bin/python3

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
import math
import numpy as np
import tf


class OccupancyToCloud:
    def __init__(self):
        self.BOX_HALFLENGTH = 1.0
        self.PORTION_OF_PI = 3.0 / 4.0
        self.TOPIC_LOCAL_MAP = "/move_base/local_costmap/costmap"
        self.TOPIC_MAP_CLOUD = "/map/cloud"
        self.map = OccupancyGrid()
        self.map_grid = None
        self.point_cloud = PointCloud2()

        self.sub_map = rospy.Subscriber(
            self.TOPIC_LOCAL_MAP, OccupancyGrid, self.callback_map
        )
        self.pub_point = rospy.Publisher(
            self.TOPIC_MAP_CLOUD, PointCloud2, queue_size=1
        )

    def callback_map(self, data):
        self.map = data
        map_data = np.array(self.map.data)
        map_Ox = self.map.info.origin.position.x
        map_Oy = self.map.info.origin.position.y
        print(self.map.info.origin)

        print("Occupied: ", np.count_nonzero(map_data >= 90))
        print("Free: ", np.count_nonzero(map_data < 90))
        # print("Total: ", data.data.count(100) + data.data.count(0) + data.data.count(-1))
        # print(map_data)

        print("Map Origin: ", map_Ox, map_Oy, " wrt ", self.map.header.frame_id)
        self.map_res = self.map.info.resolution
        map_width = self.map.info.width
        map_height = self.map.info.height
        print("Map: ", self.map.info.width, self.map.info.height)
        print("Map res: ", self.map.info.resolution)
        print(map_data.shape)
        self.map_grid = np.reshape(map_data, (map_width, map_height))
        self.map_origin = np.array([map_Ox, map_Oy])

    def bounded_angle(self, angle):
        angle = angle % (np.pi * 2)
        if angle > np.pi:
            angle = -np.pi + (angle - np.pi)
        if angle < -np.pi:
            angle = np.pi + (angle + np.pi)
        return angle

    def run(self, trans, rot):
        pointcloud = PointCloud2()

        rot_matrix = tft.quaternion_matrix(rot)
        trans_matrix = tft.translation_matrix(trans)

        yaw = tft.euler_from_quaternion(rot)[2]

        print(yaw)
        # print(rot_matrix.shape, trans_matrix.shape)
        final_matrix = np.matmul(trans_matrix, rot_matrix)
        # print(final_matrix)

        x_map_to_chassis = trans[0] - self.map_origin[0]
        y_map_to_chassis = trans[1] - self.map_origin[1]
        # print("Map to Chassis: ", x_map_to_chassis, y_map_to_chassis)

        x_odom_to_robot = trans[0]
        y_odom_to_robot = trans[1]

        x_odom_to_map = self.map_origin[0]
        y_odom_to_map = self.map_origin[1]

        top_left_map = np.array(
            [
                x_map_to_chassis - self.BOX_HALFLENGTH / 2,
                y_map_to_chassis - self.BOX_HALFLENGTH / 2,
            ]
        )
        bot_right_map = np.array(
            [
                x_map_to_chassis + self.BOX_HALFLENGTH / 2,
                y_map_to_chassis + self.BOX_HALFLENGTH / 2,
            ]
        )

        top_left_odom = np.array(
            [
                x_odom_to_robot - self.BOX_HALFLENGTH / 2,
                y_odom_to_robot - self.BOX_HALFLENGTH / 2,
            ]
        )
        bot_right_odom = np.array(
            [
                x_odom_to_robot + self.BOX_HALFLENGTH / 2,
                y_odom_to_robot + self.BOX_HALFLENGTH / 2,
            ]
        )

        index_top_left = np.array(
            [int(top_left_map[0] / self.map_res), int((top_left_map[1]) / self.map_res)]
        )
        index_bot_right = np.array(
            [
                int(bot_right_map[0] / self.map_res),
                int((bot_right_map[1]) / self.map_res),
            ]
        )
        # print("Index Top Left: ", index_top_left)
        # print("Index Top Right: ", index_top_right)
        # print("Index Bot Left: ", index_bot_left)
        # print("Index Bot Right: ", index_bot_right)

        box_grid = self.map_grid[
            index_top_left[0] : index_bot_right[0],
            index_top_left[1] : index_bot_right[1],
        ]
        # print("Shape: ", box_grid.shape)

        header = Header()
        header.frame_id = "/odom"
        # header.frame_id = "/front_laser"

        header.stamp = rospy.Time.now()

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        points = []

        # Origin
        # points.append([self.map_origin[0], self.map_origin[1], 0, 1])
        # points.append([top_left_odom[0], top_left_odom[1], 0, 1])
        # points.append([bot_right_odom[0], bot_right_odom[1], 0, 1])

        # points.append([index_bot_right[0]*self.map_res, index_bot_right[1]*self.map_res, 0, 1])

        half_fov = np.pi * self.PORTION_OF_PI
        L_bound = self.bounded_angle(half_fov + yaw)
        R_bound = self.bounded_angle(-half_fov + yaw)

        sparsity = 0.05
        points_angle = np.arange(-np.pi, np.pi, sparsity)
        points_dist = np.inf * np.ones(points_angle.shape)
        points_coord = np.zeros((points_angle.shape[0], 2))

        print("Points: ", len(points_angle))
        # print("L_R_bound, ", round(L_bound,2), round(R_bound,2))
        mid_index = np.array([int(box_grid.shape[0] / 2), int(box_grid.shape[1] / 2)])
        for i in range(box_grid.shape[0]):  # Rows
            for j in range(box_grid.shape[1]):
                heading = np.arctan2(j - mid_index[1], i - mid_index[0])
                idx = np.argmin(np.abs(points_angle - heading))

                ## Distance
                dist = np.sqrt((j - mid_index[1]) ** 2 + (i - mid_index[0]) ** 2)
                is_smaller = dist < points_dist[idx]
                ## Check for bound
                is_bounded = False
                if L_bound >= 0 and R_bound >= 0:  # 1
                    is_bounded = heading > L_bound and heading < R_bound
                elif L_bound < 0 and R_bound >= 0:  #  2
                    is_bounded = (heading > L_bound and heading < 0) or (
                        heading >= 0 and heading < R_bound
                    )
                elif L_bound >= 0 and R_bound < 0:  # 3
                    is_bounded = (heading > L_bound and heading <= np.pi) or (
                        heading >= -np.pi and heading < R_bound
                    )
                else:  # 4  L_bound < 0 and R_bound < 0
                    is_bounded = heading > L_bound and heading < R_bound

                if (
                    box_grid[j, i] == 100 and is_bounded and is_smaller
                ):  # Convert to Odom
                    points_coord[idx][0] = (
                        i * self.map_res + top_left_map[0] + self.map_origin[0]
                    )
                    points_coord[idx][1] = (
                        j * self.map_res + top_left_map[1] + self.map_origin[1]
                    )
                    points_dist[idx] = dist

                # if box_grid[j,i] == 100 and is_bounded:
                # x = i*self.map_res + top_left_map[0] + self.map_origin[0]
                # y = j*self.map_res + top_left_map[1] + self.map_origin[1]
                # points.append([x, y, 0, 1])

        for i in range(len(points_angle)):
            if points_dist[i] != np.inf:
                x = points_coord[i][0]
                y = points_coord[i][1]
                points.append([x, y, 0, 1])

        pointcloud = pc2.create_cloud(header, fields, points)
        self.pub_point.publish(pointcloud)
        print(len(points))


if __name__ == "__main__":
    rospy.init_node("map_to_cloud")
    rospy.loginfo("Map to Cloud")
    l = OccupancyToCloud()
    pause = rospy.Rate(20)

    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "/odom", "/base_link", rospy.Time(0)
            )
            l.run(trans, rot)
        except Exception as e:
            print(e)
            continue
        rospy.loginfo("Map->Cloud")
        pause.sleep()
