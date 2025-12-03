import rospy
import json
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
import sensor_msgs.point_cloud2 as pc2
import threading
import eventlet

# Initialize Flask and SocketIO
app = Flask(__name__, template_folder='../frontend', static_folder='../frontend/static')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

# Global variables to store latest data
latest_global_points = None
latest_localization = None
latest_cur_scan = None
latest_path = None
latest_cloud_registered = None

# ROS callbacks
def global_points_callback(msg):
    global latest_global_points
    points = []
    # Parse PointCloud2
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append({"x": p[0], "y": p[1], "z": p[2]})
    
    # Sample if too large (limit to 10000 points for performance)
    if len(points) > 10000:
        step = len(points) // 10000
        points = points[::step]
        
    latest_global_points = points
    socketio.emit('global_points', {'points': points})

def localization_callback(msg):
    global latest_localization
    pose = msg.pose.pose
    position = {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z
    }
    orientation = {
        "x": pose.orientation.x,
        "y": pose.orientation.y,
        "z": pose.orientation.z,
        "w": pose.orientation.w
    }
    latest_localization = {'position': position, 'orientation': orientation}
    socketio.emit('localization', latest_localization)

def cur_scan_callback(msg):
    global latest_cur_scan
    points = []
    # Parse PointCloud2
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append({"x": p[0], "y": p[1], "z": p[2]})
        
    # Sample if too large
    if len(points) > 5000:
        step = len(points) // 5000
        points = points[::step]
        
    latest_cur_scan = points
    socketio.emit('cur_scan', {'points': points})

def cloud_registered_callback(msg):
    global latest_cloud_registered
    points = []
    # Parse PointCloud2
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append({"x": p[0], "y": p[1], "z": p[2]})
        
    # Sample if too large
    if len(points) > 10000:
        step = len(points) // 10000
        points = points[::step]
        
    latest_cloud_registered = points
    socketio.emit('cloud_registered', {'points': points})

def path_callback(msg):
    global latest_path
    poses = []
    for pose_stamped in msg.poses:
        p = pose_stamped.pose.position
        poses.append({"x": p.x, "y": p.y, "z": p.z})
        
    latest_path = poses
    socketio.emit('path', {'path': poses})

# ROS thread
def ros_thread():
    # disable_signals=True allows running in a separate thread
    rospy.init_node('web_visualizer_backend', anonymous=True, disable_signals=True)
    
    rospy.Subscriber('/global_points', PointCloud2, global_points_callback)
    rospy.Subscriber('/localization', Odometry, localization_callback)
    rospy.Subscriber('/cur_scan_in_map', PointCloud2, cur_scan_callback)
    rospy.Subscriber('/cloud_registered', PointCloud2, cloud_registered_callback)
    rospy.Subscriber('/pct_path', Path, path_callback)
    
    rospy.spin()

# Routes
@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    # Send latest data if available
    if latest_global_points:
        emit('global_points', {'points': latest_global_points})
    if latest_localization:
        emit('localization', latest_localization)
    if latest_cur_scan:
        emit('cur_scan', {'points': latest_cur_scan})
    if latest_cloud_registered:
        emit('cloud_registered', {'points': latest_cloud_registered})
    if latest_path:
        emit('path', {'path': latest_path})

if __name__ == '__main__':
    # Start ROS thread
    threading.Thread(target=ros_thread, daemon=True).start()
    
    # Start Flask-SocketIO server
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
