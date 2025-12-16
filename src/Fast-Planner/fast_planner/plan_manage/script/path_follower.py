#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')
        
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 0.2)
        self.global_path = None
        self.odom_pose = None
        
        # 订阅全局路径和里程计
        # 请确保 /path 的 frame_id 与 Fast-Planner 的 frame_id 一致 (通常是 world 或 map)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        # 发布给 Fast-Planner 的目标话题
        # Fast-Planner 期望收到一个 Path 消息，但实际上只取第一个点作为当前目标
        self.goal_pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=1)
        self.target_point_pub = rospy.Publisher('/target_point', Point, queue_size=1)
        
        # 控制频率 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Path Follower Initialized. Waiting for path and odom...")

    def path_cb(self, msg):
        self.global_path = msg
        # rospy.loginfo("Received global path with %d poses", len(msg.poses))

    def odom_cb(self, msg):
        self.odom_pose = msg.pose.pose

    def control_loop(self, event):
        if self.global_path is None or self.odom_pose is None:
            return
            
        if len(self.global_path.poses) == 0:
            return

        # 1. 找到路径上离机器人最近的点
        curr_pos = np.array([self.odom_pose.position.x, self.odom_pose.position.y])
        min_dist = float('inf')
        closest_idx = -1
        
        # 简单搜索 (如果路径很长，可以优化这里)
        for i, pose_stamped in enumerate(self.global_path.poses):
            path_pos = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
            dist = np.linalg.norm(curr_pos - path_pos)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        if closest_idx == -1:
            return
            
        # 2. 向前搜索 Lookahead Distance (前瞻距离)
        lookahead_idx = closest_idx
        dist_accum = 0.0
        
        while lookahead_idx < len(self.global_path.poses) - 1:
            p1 = np.array([self.global_path.poses[lookahead_idx].pose.position.x, 
                           self.global_path.poses[lookahead_idx].pose.position.y])
            p2 = np.array([self.global_path.poses[lookahead_idx+1].pose.position.x, 
                           self.global_path.poses[lookahead_idx+1].pose.position.y])
            
            d = np.linalg.norm(p2 - p1)
            dist_accum += d
            
            if dist_accum >= self.lookahead_dist:
                break
            
            lookahead_idx += 1
            
        target_pose = self.global_path.poses[lookahead_idx]
        
        # 3. 发布目标给 Fast-Planner
        msg = Path()
        msg.header = target_pose.header # 继承 Path 的 frame_id
        msg.poses.append(target_pose)
        
        self.goal_pub.publish(msg)
        pt = Point()
        pt.x = target_pose.pose.position.x
        pt.y = target_pose.pose.position.y
        pt.z = target_pose.pose.position.z
        self.target_point_pub.publish(pt)

if __name__ == '__main__':
    try:
        PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
