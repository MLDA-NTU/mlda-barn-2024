#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
from std_msgs.msg import Header
import laser_geometry.laser_geometry as lg
import numpy as np
import tf

class LaserScanToPointCloud():
    def __init__(self):
        self.SAFE = 1.5
        self.TOPIC_LASER_SCAN = "/front/scan"
        self.TOPIC_POINT_CLOUD_LASER = "/front/laser/cloud"
        self.TOPIC_POINT_CLOUD_ODOM = "/front/odom/cloud"

        self.laser_scan = LaserScan()
        self.point_cloud = PointCloud2()
        self.laser_projector = lg.LaserProjection()
        self.sub_laser_scan = rospy.Subscriber(self.TOPIC_LASER_SCAN, LaserScan, self.callback_laser_scan)
        self.pub_point_cloud = rospy.Publisher(self.TOPIC_POINT_CLOUD_LASER, PointCloud2, queue_size=1)

        self.pub_point = rospy.Publisher(self.TOPIC_POINT_CLOUD_ODOM, PointCloud2, queue_size=1)
    
    def callback_laser_scan(self,data):
        self.laser_scan = data
        # print("LS: ", data.header.frame_id)

        # filtered = []
        # boundary = 0.5 # [m]
        # for i in range(len(self.laser_scan.ranges)):
        #     if self.laser_scan.ranges[i] > boundary:
        #         self.laser_scan.ranges[i] = self.laser_scan.range_max

        # self.laser_scan.ranges = filtered

        self.point_cloud = self.laser_projector.projectLaser(self.laser_scan)
        # print("PC: ", self.point_cloud.header.frame_id)
        # print("PC type: ", type(self.point_cloud))
        # print("H: ", self.point_cloud.height)
        # print("W: ", self.point_cloud.width)
        # print("Field: ", self.point_cloud.fields)
        # print("Point Step: ", self.point_cloud.point_step)
        # print("Row Step: ", self.point_cloud.row_step)
        # print("Data: ", type(self.point_cloud.data))
        self.pub_point_cloud.publish(self.point_cloud)
        pass
    def run(self, trans, rot):
        point_generator = pc2.read_points(self.point_cloud)
        count = 0
        pointc = PointCloud2()

        rot_matrix = tft.quaternion_matrix(rot)
        trans_matrix = tft.translation_matrix(trans)
        # print(rot_matrix.shape, trans_matrix.shape)
        final_matrix = np.matmul(trans_matrix, rot_matrix)
        # print(final_matrix)


        header = Header()
        header.frame_id = "/odom"
        # header.frame_id = "/front_laser"

        header.stamp = rospy.Time.now()

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1), 
                  PointField('intensity', 12, PointField.FLOAT32, 1)]
        points = []

        count = 0
        for point in point_generator:
            if count % 15 == 0:
                p_pc_from_laser = np.array([point[0], point[1], point[2], 1])
                p_pc_from_odom = np.matmul(final_matrix, p_pc_from_laser)
                if (p_pc_from_odom[0] - trans[0])**2 + (p_pc_from_odom[1] - trans[1])**2 < self.SAFE**2:
                    points.append(p_pc_from_odom)
            count += 1
        pointc = pc2.create_cloud(header, fields, points)
        self.pub_point.publish(pointc)
        print(len(points))
        # print("Pub")

if __name__ == "__main__":
    rospy.init_node("laser_scan_to_point_cloud")
    rospy.loginfo("Laser Scan to Point Cloud Node")
    l = LaserScanToPointCloud()

    listener = tf.TransformListener()

    pause = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/front_laser', rospy.Time(0))
            l.run(trans,rot)
        except Exception as e: 
            print(e)
            continue
        pause.sleep()