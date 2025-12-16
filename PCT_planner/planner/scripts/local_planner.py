#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import math
import time
import json
import uuid
import threading
 
try:
    import websocket
except Exception:
    websocket = None


class LocalPlanner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("local_planner")

        # 参数设置
        self.lookahead_distance = 0.9 # Reduced from 1.0 to 0.5 to reduce corner cutting
        self.max_speed = rospy.get_param("~max_speed", 1.5)
        self.max_walk_speed = rospy.get_param("~max_walk_speed", 0.6)
        self.kp_angular = rospy.get_param("~kp_angular", 0.45) # Increased from 0.4 to 0.8 for faster turning response
        self.last_speed = 0.0
        self.MAX_ACCEL = 0.3
        self.k_dist = rospy.get_param("~k_dist", 0.8)
        
        self.v_bias = rospy.get_param("~v_bias", 0.3)
        self.MAX_ACCEL_CLIMB = rospy.get_param("~max_accel_climb", 0.6)
        self.min_stair_speed = rospy.get_param("~min_stair_speed", 1.5)
        self.accid = "WF_TRON1A_316"
        self.ws_server_uri = rospy.get_param("~ws_server_uri", "ws://10.192.1.2:5000")
        self.enable_stair_ws = rospy.get_param("~enable_stair_ws", True) and websocket is not None
        self.dz_on = rospy.get_param("~dz_on", 0.05)
        self.dz_off = rospy.get_param("~dz_off", 0.02)
        self.z_window_distance = rospy.get_param("~z_window_distance", 0.8)
        self.z_window_min_points = rospy.get_param("~z_window_min_points", 5)
        self.stair_mode_debounce = rospy.get_param("~stair_mode_debounce", 1.5)
        self.stair_mode_min_hold = rospy.get_param("~stair_mode_min_hold", 4.0)
        self.stair_mode_min_hold_distance = rospy.get_param("~stair_mode_min_hold_distance", 0.6)
        self.last_mode_change_ts = 0.0
        self.last_enable_ts = 0.0
        self.last_enable_pos = None
        self.close_hold_start_idx = None
        self.front_distance = float("inf")
        self.obstacle_slow_start = rospy.get_param("~obstacle_slow_start", 1.5)
        self.obstacle_stop_dist = rospy.get_param("~obstacle_stop_dist", 0.5)
        self.up_obstacle_slow_start = rospy.get_param("~up_obstacle_slow_start", 0.75)
        self.flat_z_threshold = rospy.get_param("~flat_z_threshold", 0.05)
        self.stair_mode_enabled = False
        self.ws_open = False
        self.is_moving = True  # Default to allowing movement, can be controlled via topic
        
        # 发布控制指令
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # 订阅全局路径和机器人位姿
        self.current_path = None
        self.current_pose = None
        rospy.Subscriber("/pct_path", Path, self.path_callback)
        rospy.Subscriber("/localization", Odometry, self.odom_callback)
        rospy.Subscriber("/motion_control", Bool, self.motion_control_callback) # Topic to control start/stop
        rospy.Subscriber("/front_distance", String, self.front_distance_callback)
        self.target_pub = rospy.Publisher("/target_point", Point, queue_size=1)
        self.arrival_pub = rospy.Publisher("/arrival_status", String, queue_size=1) # Publisher for arrival status
        if self.enable_stair_ws:
            self.ws_app = websocket.WebSocketApp(self.ws_server_uri, on_open=self.on_open, on_close=self.on_close)
            threading.Thread(target=self.ws_app.run_forever, daemon=True).start()

    def path_callback(self, msg):
        """接收全局路径并转换为坐标点列表"""
        # 创建停止指令（假设已正确初始化速度值为0）
        twist = Twist()  
        # 如果需要显式设置零速度：
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # 计算每次发布的间隔（0.1秒）
        publish_rate = 10  # 赫兹
        interval = 1.0 / publish_rate

        # 连续发布10次
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            time.sleep(interval)

        # 清空当前路径
        self.current_path = None
        self.current_path = [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in msg.poses
        ]
        self.close_hold_start_idx = None
        rospy.loginfo("Received new path with {} points".format(len(self.current_path)))

    def odom_callback(self, msg):
        """获取当前机器人位姿"""
        self.current_pose = msg.pose.pose

    def motion_control_callback(self, msg):
        """Control motion start/stop"""
        self.is_moving = msg.data
        if not self.is_moving:
             # If stopped, publish zero velocity immediately
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Motion stopped by user command.")
        else:
            rospy.loginfo("Motion started by user command.")

    def front_distance_callback(self, msg):
        data = msg.data.strip()
        if data.lower() == "inf":
            self.front_distance = float("inf")
            return
        try:
            self.front_distance = float(data)
        except ValueError:
            self.front_distance = float("inf")

    def send_request(self, title, data=None):
        if data is None:
            data = {}
        message = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": data,
        }
        if self.enable_stair_ws and getattr(self, "ws_app", None) is not None and self.ws_open:
            try:
                self.ws_app.send(json.dumps(message))
            except Exception:
                pass

    def compute_z_delta_ahead(self, start_idx):
        if start_idx < 0 or self.current_path is None or len(self.current_path) == 0:
            return 0.0, 0
        z_start = float(self.current_path[start_idx][2])
        cum_dist = 0.0
        count = 1
        prev_xy = np.array(self.current_path[start_idx][:2])
        end_idx = start_idx
        for i in range(start_idx + 1, len(self.current_path)):
            cur_xy = np.array(self.current_path[i][:2])
            step = np.linalg.norm(cur_xy - prev_xy)
            cum_dist += step
            count += 1
            end_idx = i
            prev_xy = cur_xy
            if cum_dist >= self.z_window_distance:
                break
        z_end = float(self.current_path[end_idx][2])
        return (z_end - z_start), count

    def compute_path_distance(self, start_idx, end_idx):
        if self.current_path is None or len(self.current_path) == 0:
            return 0.0
        n = len(self.current_path)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx = max(0, min(end_idx, n - 1))
        if end_idx <= start_idx:
            return 0.0
        d = 0.0
        prev_xy = np.array(self.current_path[start_idx][:2])
        for i in range(start_idx + 1, end_idx + 1):
            cur_xy = np.array(self.current_path[i][:2])
            d += np.linalg.norm(cur_xy - prev_xy)
            prev_xy = cur_xy
        return d

    def on_open(self, ws):
        self.ws_open = True

    def on_close(self, ws, close_status_code, close_msg):
        self.ws_open = False

    def get_closest_point(self):
        """找到路径中距离机器人最近的点的索引 (使用欧式距离)"""
        if not self.current_path or self.current_pose is None:
            return -1

        robot_pos = np.array([
            self.current_pose.position.x, 
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        distances = [
            np.linalg.norm(robot_pos - np.array(p))
            for p in self.current_path
        ]
        return np.argmin(distances)

    def find_target_point(self, closest_idx):
        """根据前瞻距离选择目标点 (使用欧式距离)"""
        if closest_idx < 0 or closest_idx >= len(self.current_path)-1:
            return None

        # 从最近点开始向后搜索
        for i in range(closest_idx, len(self.current_path)):
            target = np.array(self.current_path[i])
            robot_pos = np.array([
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z
            ])
            # 使用欧式距离判断前瞻
            distance = np.linalg.norm(target - robot_pos)
            if distance >= self.lookahead_distance:
                return target

        # 如果所有点都小于前瞻距离，选择终点
        return np.array(self.current_path[-1])

    def calculate_velocity(self, target):
        """Pure Pursuit转向计算"""
        # 获取机器人航向角
        (_, _, yaw) = euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y,
                                    self.current_pose.orientation.z, self.current_pose.orientation.w])

        # 计算目标点在机器人坐标系中的位置
        robot_pos = np.array([
            self.current_pose.position.x, 
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        # Use 3D target for calculation if available, but control is still 2D (linear.x, angular.z)
        # Target from find_target_point is already 3D (x, y, z)
        delta_3d = target - robot_pos
        
        # Calculate yaw difference using 2D projection (navigation is still planar-ish)
        angle_diff = math.atan2(delta_3d[1], delta_3d[0]) - yaw
        
        # 归一化角度到[-pi, pi]
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        angular_speed = self.kp_angular * angle_diff

        # Use Euclidean distance (3D) for speed control
        dist = np.linalg.norm(delta_3d)
        
        linear_speed_raw = self.k_dist * dist + self.v_bias
        turning_scale = max(0.3, 1.0 - 0.5 * abs(angle_diff))
        linear_speed = min(self.max_speed, linear_speed_raw * turning_scale)
        
        # 优化：如果角度偏差过大，优先原地旋转 (Turn in place)
        if abs(angle_diff) > math.pi / 4.0: # 45 degrees
            linear_speed = 0.0
            # 确保有足够的旋转速度
            if abs(angular_speed) < 0.5:
                angular_speed = np.sign(angular_speed) * 0.5
                
        if not getattr(self, "stair_mode_enabled", False):
            linear_speed = min(linear_speed, self.max_walk_speed)
        if getattr(self, "stair_mode_enabled", False):
            linear_speed = max(linear_speed, self.min_stair_speed)
        return angular_speed, linear_speed

    def run(self):
        rate = rospy.Rate(10)  # 10Hz控制频率
        while not rospy.is_shutdown():
            if self.current_path is None or self.current_pose is None:
                rate.sleep()
                continue

            # 路径跟踪逻辑
            closest_idx = self.get_closest_point()
            target = self.find_target_point(closest_idx)
            
            # 检查是否到达终点 (使用3D距离)
            if self.current_path is not None and len(self.current_path) > 0:
                end_point = np.array(self.current_path[-1])
                robot_pos_3d = np.array([
                    self.current_pose.position.x, 
                    self.current_pose.position.y,
                    self.current_pose.position.z
                ])
                dist_to_goal = np.linalg.norm(end_point - robot_pos_3d)
                
                if dist_to_goal < 0.3: # 到达阈值 0.3m
                    # 发布停止指令
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
                    rospy.loginfo(f"Arrived at goal (dist={dist_to_goal:.3f}).")
                    self.arrival_pub.publish("arrived")
                    self.current_path = None # 清除路径以停止跟踪
                    rate.sleep()
                    continue

            delta_total, n_points = self.compute_z_delta_ahead(closest_idx)
            if target is not None:
                point_msg = Point(x=target[0], y=target[1], z=target[2])
                self.target_pub.publish(point_msg)
                robot_pos_dbg = np.array([self.current_pose.position.x, self.current_pose.position.y])
                dist_dbg = np.linalg.norm(np.array([target[0], target[1]]) - robot_pos_dbg)
                rospy.loginfo(f"lookahead target: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}, dist={dist_dbg:.3f}")
                if self.enable_stair_ws:
                    desired = self.stair_mode_enabled
                    now_ts = time.time()
                    rospy.loginfo(
                        f"stair judge: n={n_points}, z_delta={delta_total:.3f}, enabled={self.stair_mode_enabled}, desired={desired}, "
                        f"dz_on={self.dz_on:.3f}, dz_off={self.dz_off:.3f}, debounce_dt={now_ts - self.last_mode_change_ts:.2f}, hold_dt={now_ts - self.last_enable_ts:.2f}, window={self.z_window_distance:.2f}m"
                    )
                    if n_points >= self.z_window_min_points:
                        if not desired:
                            if delta_total >= self.dz_on and (now_ts - self.last_mode_change_ts) >= self.stair_mode_debounce:
                                desired = True
                        else:
                            hold_time_ok = (now_ts - self.last_enable_ts) >= self.stair_mode_min_hold
                            pre_close_ok = abs(delta_total) <= self.dz_off and hold_time_ok and (now_ts - self.last_mode_change_ts) >= self.stair_mode_debounce
                            if pre_close_ok:
                                if self.close_hold_start_idx is None:
                                    self.close_hold_start_idx = closest_idx
                                path_hold_dist = self.compute_path_distance(self.close_hold_start_idx, closest_idx)
                                rospy.loginfo(f"stair close-hold: path_dist={path_hold_dist:.3f}, need>={self.stair_mode_min_hold_distance:.3f}")
                                if path_hold_dist >= self.stair_mode_min_hold_distance:
                                    desired = False
                            else:
                                self.close_hold_start_idx = None
                    if desired != self.stair_mode_enabled:
                        rospy.loginfo(f"stair mode change: {'enable' if desired else 'disable'}")
                        self.send_request("request_stair_mode", {"enable": desired})
                        self.stair_mode_enabled = desired
                        self.last_mode_change_ts = now_ts
                        if desired:
                            self.last_enable_ts = now_ts
                            self.last_enable_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
                            self.close_hold_start_idx = None
                        else:
                            self.close_hold_start_idx = None
                
            if target is None:
                # 发布停止指令
                # (This block might be redundant now if arrival logic above handles it, 
                # but kept for safety if lookahead fails before arrival)
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("The mission has completed, no target found.")
                self.arrival_pub.publish("arrived") # Publish arrival message
                # break
                

                rate.sleep()
                continue

            if not self.is_moving:
                # If motion is disabled, skip velocity calculation and publishing (or publish zero)
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
                continue

            angular_speed, linear_speed= self.calculate_velocity(target)
                        
            if closest_idx >= len(self.current_path)-3:
                linear_speed *= 0.3
                
            obstacle_d = self.front_distance
            if obstacle_d < self.obstacle_slow_start:
                if abs(delta_total) < self.flat_z_threshold:
                    motion_profile = "flat"
                    slow_start = self.obstacle_slow_start
                    stop_dist = self.obstacle_stop_dist
                elif delta_total > 0.0:
                    motion_profile = "up"
                    slow_start = self.up_obstacle_slow_start
                    stop_dist = 0.0
                else:
                    motion_profile = "down"
                    slow_start = self.obstacle_slow_start
                    stop_dist = self.obstacle_stop_dist
                if obstacle_d <= stop_dist:
                    ratio = 0.0
                else:
                    denom = slow_start - stop_dist
                    if denom <= 0.0:
                        ratio = 0.0
                    else:
                        ratio = (obstacle_d - stop_dist) / denom
                        ratio = max(0.0, min(1.0, ratio))
                if motion_profile == "down":
                    target_end = -0.25
                    linear_speed = target_end + (linear_speed - target_end) * ratio
                else:
                    linear_speed = linear_speed * ratio

            delta_speed = linear_speed - self.last_speed
            accel_limit = self.MAX_ACCEL
            if getattr(self, "stair_mode_enabled", False):
                accel_limit = max(accel_limit, self.MAX_ACCEL_CLIMB)
            if abs(delta_speed) > accel_limit * 0.1:
                linear_speed = self.last_speed + np.sign(delta_speed) * accel_limit * 0.1

            # 强制限制最大速度（防止模式切换瞬间速度过高）
            current_max_speed = self.max_speed
            if not getattr(self, "stair_mode_enabled", False):
                current_max_speed = min(current_max_speed, self.max_walk_speed)
            linear_speed = min(linear_speed, current_max_speed)

            self.last_speed = linear_speed
            
            # 发布指令
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)

            rate.sleep()

if __name__ == "__main__":
    planner = LocalPlanner()
    planner.run()
