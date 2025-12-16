#!/usr/bin/env python3
# coding=utf-8

import rospy
import numpy as np
from livox_ros_driver2.msg import CustomMsg
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class Livox3DFrontDetector:
    def __init__(self):
        rospy.init_node("livox_3d_front_detector", anonymous=True)

        rospy.Subscriber("/livox/lidar", CustomMsg, self.cloud_callback)

        self.pub = rospy.Publisher("/front_distance", String, queue_size=5)
        self.cloud_pub = rospy.Publisher("/front_region_cloud", PointCloud2, queue_size=1)

        rospy.loginfo("✔ Livox 3D 前方体积检测节点已启动，订阅 /livox/lidar")     
        rospy.spin()

    def cloud_callback(self, cloud_msg):

        points_np = np.array([(p.x, p.y, p.z) for p in cloud_msg.points], dtype=np.float32)

        if points_np.size == 0:
            return

        X_MIN, X_MAX = 0.29, 2.0     # 前方 0.1 - 2m
        Y_MIN, Y_MAX = -0.1, 0.1    # 左右 ±0.3m
        Z_MIN, Z_MAX = -0.2, 0.3    # 高度范围

        x = points_np[:, 0]
        y = points_np[:, 1]
        z = points_np[:, 2]

        mask = (
            (x > X_MIN) & (x < X_MAX) &
            (y > Y_MIN) & (y < Y_MAX) &
            (z > Z_MIN) & (z < Z_MAX)
        )

        region_points = points_np[mask]

        if len(region_points) == 0:
            header = cloud_msg.header
            header.frame_id = "body"
            empty_cloud = pc2.create_cloud_xyz32(header, [])
            self.cloud_pub.publish(empty_cloud)
            self.pub.publish("inf")
            print("no obstacle")
            return

        centroid = region_points.mean(axis=0)
        dist = float(centroid[0])

        self.pub.publish(str(round(dist, 3)))
        print("Front Obstacle Distance:", round(dist, 3))
        header = cloud_msg.header
        header.frame_id = "body"
        filtered_cloud = pc2.create_cloud_xyz32(header, region_points.tolist())
        self.cloud_pub.publish(filtered_cloud)


if __name__ == '__main__':
    Livox3DFrontDetector()
