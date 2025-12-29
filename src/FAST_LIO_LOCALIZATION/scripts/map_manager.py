#!/usr/bin/env python3
# coding=utf8

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import ros_numpy
import os
import glob
import argparse
import sys

class MapManager:
    def __init__(self):
        rospy.init_node('map_manager')
        
        # Parameters
        self.map_dir = rospy.get_param('~map_dir', '')  # Directory containing split PCD files
        self.block_size_x = rospy.get_param('~block_size_x', 50.0)
        self.block_size_y = rospy.get_param('~block_size_y', 50.0)
        self.block_size_z = rospy.get_param('~block_size_z', 50.0)
        self.buffer_radius = rospy.get_param('~buffer_radius', 1)
        
        if not self.map_dir or not os.path.exists(self.map_dir):
            rospy.logerr("Map directory not found: {}".format(self.map_dir))
            return

        rospy.loginfo("Map Manager initialized. Watching directory: {}".format(self.map_dir))
        rospy.loginfo("Block Size: ({}, {}, {})".format(self.block_size_x, self.block_size_y, self.block_size_z))
        
        self.current_block = None
        self.loaded_blocks = {} # Cache for loaded blocks: key=(x,y), value=pcd_points
        self.pub_map = rospy.Publisher('/map', PointCloud2, queue_size=1, latch=True)
        # self.sub_odom = rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        self.sub_localization = rospy.Subscriber('/localization', Odometry, self.localization_cb)
        
        rospy.loginfo("Waiting for Localization result...")

    def get_block_filename(self, x, y, z):
        return os.path.join(self.map_dir, "block_{}_{}_{}.pcd".format(x, y, z))

    def load_block(self, x, y, z):
        key = (x, y, z)
        if key in self.loaded_blocks:
            return self.loaded_blocks[key]
            
        filename = self.get_block_filename(x, y, z)
        if os.path.exists(filename):
            try:
                pcd = o3d.io.read_point_cloud(filename)
                points = np.asarray(pcd.points)
                if len(points) > 0:
                    self.loaded_blocks[key] = points
                    return points
            except Exception as e:
                rospy.logwarn("Failed to load block {}: {}".format(filename, e))
        return None

    # def odom_cb(self, msg):
    #     pos = msg.pose.pose.position
    #     idx_x = int(np.floor(pos.x / self.block_size))
    #     idx_y = int(np.floor(pos.y / self.block_size))
    #     idx_z = int(np.floor(pos.z / self.block_size))
    #     
    #     if self.current_block != (idx_x, idx_y, idx_z):
    #         self.current_block = (idx_x, idx_y, idx_z)
    #         self.update_local_map(idx_x, idx_y, idx_z)

    def localization_cb(self, msg):
        pos = msg.pose.pose.position
        idx_x = int(np.floor(pos.x / self.block_size_x))
        idx_y = int(np.floor(pos.y / self.block_size_y))
        idx_z = int(np.floor(pos.z / self.block_size_z))
        
        if self.current_block != (idx_x, idx_y, idx_z):
            self.current_block = (idx_x, idx_y, idx_z)
            self.update_local_map(idx_x, idx_y, idx_z)
            
    def update_local_map(self, center_x, center_y, center_z):
        points_list = []
        r = self.buffer_radius
        
        # Determine needed blocks (3D grid)
        needed_keys = set()
        for x in range(center_x - r, center_x + r + 1):
            for y in range(center_y - r, center_y + r + 1):
                for z in range(center_z - r, center_z + r + 1):
                    needed_keys.add((x, y, z))
                    points = self.load_block(x, y, z)
                    if points is not None:
                        points_list.append(points)
        
        rospy.loginfo("Current block: ({}, {}, {}). Loading {} surrounding blocks.".format(center_x, center_y, center_z, len(points_list)))
        
        # Garbage collection: remove blocks that are far away
        current_keys = list(self.loaded_blocks.keys())
        for key in current_keys:
            if key not in needed_keys:
                del self.loaded_blocks[key]
        
        if not points_list:
            rospy.logwarn("No map blocks found around ({}, {}, {})".format(center_x, center_y, center_z))
            return
            
        local_points = np.vstack(points_list)
        rospy.loginfo("Publishing local map: {} points from {} blocks.".format(len(local_points), len(points_list)))
        
        # Publish
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        
        data = np.zeros(len(local_points), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        data['x'] = local_points[:, 0]
        data['y'] = local_points[:, 1]
        data['z'] = local_points[:, 2]
        
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header = header
        self.pub_map.publish(msg)

# ----------------------------------------------------------------------------------
# Preprocessing Tool
# ----------------------------------------------------------------------------------
def split_map_tool():
    parser = argparse.ArgumentParser(description="Split a large PCD map into blocks.")
    parser.add_argument("input_pcd", help="Path to input .pcd file")
    parser.add_argument("output_dir", help="Directory to save block files")
    parser.add_argument("--block_size_x", type=float, default=50.0, help="Block size X in meters")
    parser.add_argument("--block_size_y", type=float, default=50.0, help="Block size Y in meters")
    parser.add_argument("--block_size_z", type=float, default=50.0, help="Block size Z in meters")
    
    # Filter out ROS args
    args = parser.parse_args(rospy.myargv()[1:])
    
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
        print("Created output directory: {}".format(args.output_dir))
        
    print("Loading map: {}".format(args.input_pcd))
    pcd = o3d.io.read_point_cloud(args.input_pcd)
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        print("Error: Input map is empty.")
        return

    print("Map loaded. {} points. Splitting...".format(len(points)))
    
    # Calculate block indices (3D)
    indices = np.zeros((len(points), 3), dtype=int)
    indices[:, 0] = np.floor(points[:, 0] / args.block_size_x).astype(int)
    indices[:, 1] = np.floor(points[:, 1] / args.block_size_y).astype(int)
    indices[:, 2] = np.floor(points[:, 2] / args.block_size_z).astype(int)
    
    unique_blocks = np.unique(indices, axis=0)
    
    print("Found {} unique blocks.".format(len(unique_blocks)))
    
    for block in unique_blocks:
        mask = (indices[:, 0] == block[0]) & (indices[:, 1] == block[1]) & (indices[:, 2] == block[2])
        block_points = points[mask]
        
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(block_points)
        
        filename = os.path.join(args.output_dir, "block_{}_{}_{}.pcd".format(block[0], block[1], block[2]))
        o3d.io.write_point_cloud(filename, out_pcd)
        # print("Saved {}".format(filename)) # Uncomment for verbose output
        
    print("Done! Blocks saved to {}".format(args.output_dir))

if __name__ == '__main__':
    # Determine mode based on arguments
    # If launched as a ROS node, it will have __name:=...
    is_ros_node = False
    for arg in sys.argv:
        if arg.startswith('__name:='):
            is_ros_node = True
            break
            
    if is_ros_node:
        try:
            MapManager()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    else:
        # Run as a standalone script for preprocessing
        split_map_tool()
