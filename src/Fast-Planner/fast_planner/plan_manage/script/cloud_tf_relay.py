#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix

class CloudTFRelay:
    def __init__(self):
        self.input_cloud_topic = rospy.get_param('~input_cloud_topic', '/cloud_registered')
        self.transform_topic = rospy.get_param('~transform_topic', '/map_to_odom')
        self.output_cloud_topic = rospy.get_param('~output_cloud_topic', '/cloud_registered_odom')
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.R = np.eye(3, dtype=np.float32)
        self.t = np.zeros(3, dtype=np.float32)
        self.cloud_sub = rospy.Subscriber(self.input_cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)
        self.tf_sub = rospy.Subscriber(self.transform_topic, Odometry, self.tf_cb, queue_size=10)
        self.pub = rospy.Publisher(self.output_cloud_topic, PointCloud2, queue_size=1)

    def tf_cb(self, msg):
        q = msg.pose.pose.orientation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        self.R = T[:3, :3].astype(np.float32)
        p = msg.pose.pose.position
        self.t = np.array([p.x, p.y, p.z], dtype=np.float32)

    def cloud_cb(self, msg):
        if self.R is None or self.t is None:
            return
        pts_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array(list(pts_iter), dtype=np.float32)
        if pts.shape[0] == 0:
            return
        out = (pts @ self.R.T) + self.t
        points = [tuple(p) for p in out]
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        header = msg.header
        header.frame_id = self.target_frame
        header.stamp = msg.header.stamp
        cloud = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node('cloud_tf_relay')
    CloudTFRelay()
    rospy.spin()
