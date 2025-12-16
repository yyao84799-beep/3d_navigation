#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TrajToCmdVel:
    def __init__(self):
        rospy.init_node('traj_to_cmd_vel')
        
        # Parameters
        self.k_p = rospy.get_param('~k_p', 1.0)       # Position gain
        self.k_w = rospy.get_param('~k_w', 4.0)       # Angular gain
        self.max_v = rospy.get_param('~max_v', 0.3)   # Max linear speed
        self.max_w = rospy.get_param('~max_w', 0.5)   # Max angular speed
        self.use_feedforward = rospy.get_param('~use_feedforward', True) # Use velocity feedforward
        
        # State
        self.odom_pose = None
        self.odom_yaw = 0.0
        self.cmd_msg = Twist()
        
        # Publishers & Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pos_cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.cmd_cb)
        self.odom_sub = rospy.Subscriber('/localization', Odometry, self.odom_cb)
        
        rospy.loginfo("TrajToCmdVel Node Initialized")

    def odom_cb(self, msg):
        self.odom_pose = msg.pose.pose.position
        
        # Extract Yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.odom_yaw = yaw

    def cmd_cb(self, msg):
        if self.odom_pose is None:
            return
            
        # 1. Target Error
        err_x = msg.position.x - self.odom_pose.x
        err_y = msg.position.y - self.odom_pose.y
        dist = math.sqrt(err_x**2 + err_y**2)
        
        # 2. Feedforward Velocity
        # Fast-Planner gives target velocity in msg.velocity
        v_ff_x = msg.velocity.x
        v_ff_y = msg.velocity.y
        v_ff = math.sqrt(v_ff_x**2 + v_ff_y**2)
        
        # 3. Calculate Target Heading
        # For differential drive, we must face the target or the direction of motion
        if dist > 0.05:
            # If significant position error, point to target (Pure Pursuit style logic mixed with PID)
            # Or point in direction of feedforward velocity?
            # Strategy: Point towards "Target Position + Lookahead"
            # Here we simple point towards target position if error is large,
            # but if tracking well, we point in direction of velocity.
            
            # Simple approach: P-controller on position
            v_cmd_x = self.k_p * err_x
            v_cmd_y = self.k_p * err_y
            
            if self.use_feedforward:
                v_cmd_x += v_ff_x
                v_cmd_y += v_ff_y
                
            # Resultant desired velocity vector
            v_des = math.sqrt(v_cmd_x**2 + v_cmd_y**2)
            heading_des = math.atan2(v_cmd_y, v_cmd_x)
            
        else:
            # Reached target (or very close), just rotate to desired yaw if provided
            # But Fast-Planner's yaw might be independent. 
            # For diff drive, we usually stop if v is 0.
            v_des = 0.0
            heading_des = self.odom_yaw # Hold current heading
            
            if self.use_feedforward and v_ff > 0.1:
                # If path continues but we are on track
                heading_des = math.atan2(v_ff_y, v_ff_x)
                v_des = v_ff
            
        # 4. Angular Control
        # Normalize angle error
        yaw_err = heading_des - self.odom_yaw
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi
            
        w_cmd = self.k_w * yaw_err
        
        # 5. Linear Control (Project velocity onto current heading to avoid moving sideways)
        # We only move forward if we are roughly facing the target
        if abs(yaw_err) > math.pi / 2:
            # If facing backwards, maybe stop or turn in place?
            # For now, just turn
            v_out = 0.0
        else:
            # Scale linear velocity by alignment (move slower if turning)
            v_out = v_des * math.cos(yaw_err)
            
        # Saturation
        v_out = min(max(v_out, -self.max_v), self.max_v)
        w_out = min(max(w_cmd, -self.max_w), self.max_w)
        
        # Publish
        cmd = Twist()
        cmd.linear.x = v_out
        cmd.angular.z = w_out
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        TrajToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
