#!/usr/bin/env python3
import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Spiral', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\', \'Room\']')
args, _ = parser.parse_known_args(rospy.myargv(argv=sys.argv)[1:])
def goal_callback(msg):
    """接收目标点的回调函数"""
    global end_pos, end_height  # 声明修改全局变量
    # 从消息中提取目标点坐标
    end_pos = np.array([msg.point.x, msg.point.y], dtype=np.float32)
    end_height = msg.point.z
    # 触发新的路径规划
    pct_plan()

def localization_callback(msg):
    """接收定位数据的回调函数"""
    global start_pos,  start_height# 声明修改全局变量
    # 从消息中提取当前位置坐标
    start_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=np.float32)
    start_height = msg.pose.pose.position.z-1

cfg = Config()
global_z_offset = 0.25 # 用户可以通过修改此值来抬高或降低全局路径的高度

if args.scene == 'Spiral':
    tomo_file = 'spiral0.3_2'
    start_pos = np.array([-16.0, -6.0], dtype=np.float32)
    end_pos = np.array([-26.0, -5.0], dtype=np.float32)
elif args.scene == 'Building':
    tomo_file = 'building2_9'
    start_pos = np.array([11.1, 1.38], dtype=np.float32)
    end_pos = np.array([-6.0, -1.0], dtype=np.float32)
elif args.scene == 'Plaza':
    tomo_file = 'plaza3_10'
    start_pos = np.array([0.0, 0.0], dtype=np.float32)
    end_pos = np.array([1.98, 24.7], dtype=np.float32)
    start_height=0
    end_height=0
elif args.scene == 'Room':
    tomo_file = 'scans_room'
    start_pos = np.array([-3.22, -1.69], dtype=np.float32)
    end_pos = np.array([5.84, 0.462], dtype=np.float32)
    start_height=0
    end_height=0
elif args.scene == 'Stairs':
    tomo_file = 'stairs_best'
    start_pos = np.array([5.06, -0.379], dtype=np.float32)
    end_pos = np.array([6.99, 8.29], dtype=np.float32)
    start_height=0
    end_height=0
elif args.scene == 'Floor':
    tomo_file = 'floor_best'
    # start_pos = np.array([2.53, 6.74], dtype=np.float32)
    end_pos = np.array([4.8, -0.526], dtype=np.float32)
    start_height=0
    end_height=0
elif args.scene == 'Common':
    tomo_file = 'scans'
    start_pos = np.array([1.21186, 0.277206], dtype=np.float32)
    end_pos = np.array([1.7279, 0.416748], dtype=np.float32)
    start_height=-0.1
    end_height=-0.2

path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

def post_process_path(traj_3d):
    if traj_3d is None or len(traj_3d) < 2:
        return traj_3d
    
    # 识别楼梯段并拉直
    # 简单逻辑：检测连续Z变化超过阈值的段，视为楼梯
    # 将该段起点和终点连线，替换中间点
    
    new_traj = []
    n = len(traj_3d)
    
    # 阈值参数
    z_diff_thresh = 0.03 # 相邻点高度差阈值，视为斜坡/楼梯趋势
    stair_segment_min_len = 3 # 至少多少个点才视为楼梯段
    
    i = 0
    while i < n:
        new_traj.append(traj_3d[i])
        
        # 检查是否开始进入楼梯段 (连续上升或下降)
        if i + 1 < n:
            dz = traj_3d[i+1][2] - traj_3d[i][2]
            if abs(dz) > z_diff_thresh:
                # 潜在楼梯段起点
                start_idx = i
                end_idx = i
                direction = np.sign(dz)
                
                # 向前搜索直到Z变化趋势停止
                # 优化：允许平坦段（平台），但如果平坦段太长（超过一定点数），则认为楼梯中断
                flat_count = 0
                max_flat_len = 3 # 允许中间最多几个点是平坦的（台阶平面），超过则视为平台
                
                for j in range(i + 1, n):
                    dz_j = traj_3d[j][2] - traj_3d[j-1][2]
                    
                    if abs(dz_j) > 0.01 and np.sign(dz_j) == direction:
                        # 继续上升/下降，重置平坦计数
                        end_idx = j
                        flat_count = 0
                    elif abs(dz_j) <= 0.01:
                        # 平坦点
                        flat_count += 1
                        if flat_count > max_flat_len:
                            # 平台太长，视为楼梯结束，回退到上一个有效爬升点
                            # end_idx 保持在进入长平台之前的最后一个点
                            break
                    else:
                        # 反向，结束
                        break
                
                # 如果段长度足够，进行直线化处理
                if end_idx - start_idx >= stair_segment_min_len:
                    p_start_raw = traj_3d[start_idx]
                    p_end_raw = traj_3d[end_idx]
                    
                    # 优化：楼梯段的起始点与终点x、y坐标变为对应段的均值，z坐标不变
                    # 提取该段的所有点
                    segment_points = traj_3d[start_idx:end_idx+1]
                    mean_x = np.mean(segment_points[:, 0])
                    mean_y = np.mean(segment_points[:, 1])
                    
                    # 构造新的起点和终点 (XY取均值，Z保持原值)
                    # 注意：这意味着整个楼梯在XY平面上变成了一个点（垂直电梯）？
                    # 用户说“侧视平行于楼梯并且正视垂直与楼梯”
                    # "x、y坐标变为对应段的均值" -> 这会让起点和终点的XY都变成同一个点，意味着楼梯是垂直上下的？
                    # 还是说：
                    # 1. 计算整个段的XY均值中心？
                    # 2. 还是说分别计算起点附近的均值和终点附近的均值？
                    # 3. 或者是想把路径在XY平面上拉成一条直线？
                    
                    # 重新阅读："楼梯段的起始点与终点x、y坐标变为对应段的均值"
                    # 字面意思：p_start.x = mean_x, p_end.x = mean_x
                    # 这样楼梯就变成垂直的了（Z变化，XY不变）。这通常不是楼梯，是电梯。
                    # 除非用户是指：将路径投影到由起点终点决定的直线上？
                    
                    # 另一种理解：用户可能想让楼梯路径在XY平面上“对齐”到楼梯的中轴线。
                    # 但如果只是简单取均值，起点和终点XY重合，路径长度为0（水平距离）。
                    # 机器人无法爬垂直墙。
                    
                    # 假设用户意图：让路径在XY平面上更直，例如取起点和终点的连线。
                    # 但用户明确说“x、y坐标变为对应段的均值”。
                    # 也许是指：将这段路径的所有点的XY都设为均值？那也是垂直的。
                    
                    # 可能用户是指：将这段路径的“方向”修正为楼梯方向？
                    # 或者用户想表达：起点和终点的XY坐标，分别取自“对应段的均值”？
                    # 也许是：起点XY = start_idx附近几个点的均值，终点XY = end_idx附近几个点的均值？
                    # 这样可以平滑起点和终点的位置。
                    
                    # 但如果按字面“变为对应段的均值”，确实会导致垂直。
                    # 让我们暂时假设用户是想把路径拉直，且对齐到某种中心。
                    # 也许是：
                    # 计算整个段的线性回归直线（在XY平面），然后把起点和终点投影到这条直线上？
                    
                    # 结合“侧视平行于楼梯并且正视垂直与楼梯”：
                    # 侧视平行 -> Z随XY线性变化（我们已经做了）。
                    # 正视垂直 -> 在XY平面上是一条直线，且垂直于楼梯台阶边缘（即沿着楼梯方向）。
                    
                    # 鉴于“x、y变为对应段均值”会导致垂直，我猜测用户可能是口误，
                    # 或者是想说：将路径点的横向偏差（偏离楼梯中轴线的量）去除，即让所有点的XY都在起点和终点的连线上。
                    # 而起点和终点的位置，可能需要优化。
                    
                    # 如果按照字面执行（垂直），机器人可能无法规划速度。
                    # 让我们尝试一种更合理的解释：
                    # 将起点和终点的XY坐标，修正为这段路径在XY平面上的拟合直线的两个端点。
                    # 或者，也许用户是指：这一段路径在XY平面上应该是一条直线，而这条直线的XY位置由该段点的均值决定？
                    # 不，一条直线由两个点决定。
                    
                    # 让我们采用“拟合直线”的思路：
                    # 1. 用该段所有点的 (x,y) 拟合一条直线 L。
                    # 2. 将 p_start 的 (x,y) 投影到 L 上作为新起点 XY。
                    # 3. 将 p_end 的 (x,y) 投影到 L 上作为新终点 XY。
                    # 4. Z 保持 p_start.z 和 p_end.z。
                    # 这样既保留了斜率，又拉直了 XY 轨迹。
                    
                    # 简单实现：直接用 p_start 和 p_end 连线作为 L。
                    # 但如果 p_start 或 p_end 刚好是歪的（噪声），直线就歪了。
                    # 取均值可能指：
                    # Start_XY_New = Mean(First N points)
                    # End_XY_New = Mean(Last N points)
                    # 这样更稳健。
                    
                    # 让我们尝试：取前3个点的均值作为起点XY，后3个点的均值作为终点XY。
                    segment_len = end_idx - start_idx
                    n_smooth = min(3, segment_len)
                    start_pts = traj_3d[start_idx : start_idx + n_smooth]
                    end_pts = traj_3d[end_idx - n_smooth + 1 : end_idx + 1]
                    
                    p_start_new = p_start_raw.copy()
                    p_end_new = p_end_raw.copy()
                    
                    p_start_new[0] = np.mean(start_pts[:, 0])
                    p_start_new[1] = np.mean(start_pts[:, 1])
                    
                    p_end_new[0] = np.mean(end_pts[:, 0])
                    p_end_new[1] = np.mean(end_pts[:, 1])

                    # 楼梯路段所有的x坐标需要更新为对应路径的x坐标的均值
                    # 意味着：整个楼梯段在X轴上的位置统一为该段所有点X坐标的平均值
                    # Y轴和Z轴保持线性插值（或原样）
                    # 注意：如果楼梯是沿着X轴走的，这样做会让楼梯变成垂直的！
                    # 假设楼梯是沿着Y轴走的，那么统一X坐标是合理的（修正横向偏差）。
                    # 但我们不知道楼梯的方向。
                    # 如果用户说“所有x坐标更新为...均值”，那么这可能是一个针对特定场景（楼梯沿Y轴）的优化。
                    # 或者，用户是指“横向坐标”（相对于路径方向的横向偏移）归零？
                    # 鉴于用户指令非常具体：“楼梯路段所有的x坐标需要更新为对应路径的x坐标的均值”
                    # 我们按指令修改：将插值点的X坐标强制设为该段所有原始点X的均值。
                    
                    segment_points = traj_3d[start_idx:end_idx+1]
                    avg_x = np.mean(segment_points[:, 0])
                    
                    # 如果我们强制 X = avg_x，那么起点和终点的 X 也必须是 avg_x，否则路径会断开。
                    p_start_new[0] = avg_x
                    p_end_new[0] = avg_x
                    
                    # Z 保持不变（楼梯的物理高度是固定的）
                    
                    segment_len = end_idx - start_idx
                    for k in range(1, segment_len + 1):
                        alpha = k / float(segment_len)
                        # p_interp = (1 - alpha) * p_start_new + alpha * p_end_new
                        # 仅对 Y 进行线性插值 (X 固定为 avg_x, Z 线性插值)
                        # 其实 p_start_new 和 p_end_new 的 X 已经是 avg_x 了，所以直接插值向量也是对的。
                        p_interp = (1 - alpha) * p_start_new + alpha * p_end_new
                        new_traj.append(p_interp)
                    
                    i = end_idx # 跳过已处理段
                else:
                    pass # 不是楼梯，继续下一个点
        
        i += 1
        
    return np.array(new_traj)

def pct_plan():


    print(start_pos, end_pos, start_height, end_height)
    traj_3d = planner.plan(start_pos, end_pos, start_height, end_height)
    
    if traj_3d is not None:
        # 后处理：楼梯直线化
        traj_3d = post_process_path(traj_3d)
        traj_3d[:, 2] += global_z_offset # 应用全局Z轴偏移
        path_pub.publish(traj2ros(traj_3d))
        print("Trajectory published")

        # Removed blocking arrival check loop to allow interrupting with new goals.
        # Arrival check is now handled by local_planner.

if __name__ == '__main__':
    rospy.init_node("pct_planner", anonymous=True)
    planner.loadTomogram(tomo_file)
    rospy.Subscriber("/localization", Odometry, localization_callback)
    rospy.Subscriber("/clicked_point", PointStamped, goal_callback)

    rospy.spin()
