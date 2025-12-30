#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class GlobalToLocalPath:
    def __init__(self):
        rospy.init_node('global_to_local_path_node', anonymous=True)

        # 参数配置
        self.lookahead_dist = 2.0  # 前瞻距离 2m
        self.path_topic = "/pct_path"
        self.odom_topic = "/localization"
        self.goal_topic = "/move_base_simple/goal"

        self.latest_path = None
        self.current_goal_idx = -1  # 当前锁定的目标点索引
        self.last_published_idx = -1 # 上一次发布的目标点索引
        self.arrival_threshold = 0.3 # 到达阈值 1.0m

        # 订阅者
        self.sub_path = rospy.Subscriber(self.path_topic, Path, self.path_callback)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        # 发布者 - 降低频率，避免高频震荡
        self.pub_goal = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback) # 10Hz

        self.robot_pose = None # 存储最新机器人位置

        rospy.loginfo(f"Initialized GlobalToLocalPath node.")
        rospy.loginfo(f"Listening to Path: {self.path_topic}")
        rospy.loginfo(f"Listening to Odom: {self.odom_topic}")
        rospy.loginfo(f"Publishing Goal: {self.goal_topic} with lookahead: {self.lookahead_dist}m")

    def path_callback(self, msg):
        # 收到新路径时更新，重置索引
        self.latest_path = msg
        self.current_goal_idx = -1 # 重置，重新寻找最近点作为起点
        self.last_published_idx = -1

    def odom_callback(self, odom_msg):
        self.robot_pose = odom_msg.pose.pose.position

    def timer_callback(self, event):
        if self.latest_path is None or len(self.latest_path.poses) == 0 or self.robot_pose is None:
            return

        # 获取机器人当前位置
        rx = self.robot_pose.x
        ry = self.robot_pose.y
        rz = self.robot_pose.z

        # 如果尚未选定目标点（刚启动或刚重置），先找到离机器人最近的点，然后往后找一个前瞻点
        if self.current_goal_idx == -1:
            min_dist_sq = float('inf')
            closest_idx = 0
            for i, pose in enumerate(self.latest_path.poses):
                px = pose.pose.position.x
                py = pose.pose.position.y
                pz = pose.pose.position.z
                d2 = (px-rx)**2 + (py-ry)**2 + (pz-rz)**2
                if d2 < min_dist_sq:
                    min_dist_sq = d2
                    closest_idx = i
            
            # 从最近点开始找第一个前瞻点
            found = False
            for i in range(closest_idx, len(self.latest_path.poses)):
                px = self.latest_path.poses[i].pose.position.x
                py = self.latest_path.poses[i].pose.position.y
                pz = self.latest_path.poses[i].pose.position.z
                dist = math.sqrt((px-rx)**2 + (py-ry)**2 + (pz-rz)**2)
                if dist >= self.lookahead_dist:
                    self.current_goal_idx = i
                    found = True
                    break
            
            if not found:
                self.current_goal_idx = len(self.latest_path.poses) - 1

        # 检查是否到达当前目标点
        # 获取当前目标点的坐标
        if self.current_goal_idx >= len(self.latest_path.poses):
             self.current_goal_idx = len(self.latest_path.poses) - 1

        curr_goal_pose = self.latest_path.poses[self.current_goal_idx].pose.position
        dist_to_goal = math.sqrt((curr_goal_pose.x - rx)**2 + 
                                 (curr_goal_pose.y - ry)**2 + 
                                 (curr_goal_pose.z - rz)**2)

        # 如果距离小于阈值，说明到达了，寻找下一个目标点
        if dist_to_goal < self.arrival_threshold:
            # 从当前目标点索引开始继续往后找，直到找到一个距离机器人 > lookahead_dist 的点
            # 或者也可以简单的：当前索引 + lookahead_step，但按距离找更稳健
            
            found_next = False
            for i in range(self.current_goal_idx, len(self.latest_path.poses)):
                px = self.latest_path.poses[i].pose.position.x
                py = self.latest_path.poses[i].pose.position.y
                pz = self.latest_path.poses[i].pose.position.z
                
                # 计算该点到机器人的距离（注意是到机器人，不是到上一个目标点）
                d = math.sqrt((px-rx)**2 + (py-ry)**2 + (pz-rz)**2)
                
                # 找到下一个前瞻点
                if d >= self.lookahead_dist:
                    self.current_goal_idx = i
                    found_next = True
                    break
            
            if not found_next:
                 self.current_goal_idx = len(self.latest_path.poses) - 1

        # 只有当目标点发生变化时才发布
        if self.current_goal_idx != self.last_published_idx:
            # 发布当前锁定的目标点
            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = self.latest_path.header.frame_id
            goal_msg.pose = self.latest_path.poses[self.current_goal_idx].pose
            
            self.pub_goal.publish(goal_msg)
            rospy.loginfo(f"Published new local goal. Index: {self.current_goal_idx}")
            
            self.last_published_idx = self.current_goal_idx

if __name__ == '__main__':
    try:
        node = GlobalToLocalPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
