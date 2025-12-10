import uvicorn
import socketio
import threading
import asyncio
import rospy
import sensor_msgs.point_cloud2 as pc2
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_matrix, quaternion_multiply

# --- ROS Configuration ---
# Global storage for latest data
latest_data = {
    "global_points": None, # Only store bytes if needed
    "localization": None,
    # "cur_scan": [], # Removed as requested by frontend simplification
    "path": [],
    "cloud_registered": [],
    "arrival_status": None,
    "waypoint_queue": [],
    "target_point": None, # New: Real-time lookahead target
    "baselink2map": None,
    "odom2map": None,
    "submap_boxes": None
}

import numpy as np
# ROS Callbacks
def target_point_callback(msg):
    latest_data["target_point"] = {"x": msg.x, "y": msg.y, "z": msg.z}

def global_points_callback(msg):
    # Check if we already have global points
    # But if latest_data["global_points"] is None, we MUST process it.
    # If it is NOT None, we can skip to save CPU if we assume map is static.
    # However, if user changes map, we should update it.
    # Let's just update it if the size changes significantly or force update?
    # Or better: Always update it, but optimize frontend transmission.
    
    points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_np = np.array(list(points_gen), dtype=np.float32)
    
    # Sampling if too large (optimized to 50k max)
    if points_np.shape[0] > 50000:
         points_np = points_np[::points_np.shape[0] // 50000]
         
    # Store as bytes for efficient transmission
    new_data = points_np.tobytes()
    
    # Update local cache but DO NOT broadcast automatically to avoid lag
    if latest_data["global_points"] != new_data:
        latest_data["global_points"] = new_data
        # We do NOT broadcast here anymore. Frontend must request it.

async def broadcast_map_update():
    # Deprecated or used only for explicit forced updates
    if 'connected_sids' in globals() and latest_data["global_points"]:
        for sid in list(connected_sids):
             await sio.emit('global_points', latest_data["global_points"], room=sid)

def localization_callback(msg):
    pose = msg.pose.pose
    position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
    orientation = {
        "x": pose.orientation.x,
        "y": pose.orientation.y,
        "z": pose.orientation.z,
        "w": pose.orientation.w
    }
    latest_data["localization"] = {"position": position, "orientation": orientation}

# def cur_scan_callback(msg):
#     # Disabled to save bandwidth
#     pass

def cloud_registered_callback(msg):
    points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_np = np.array(list(points_gen), dtype=np.float32)
    
    # Downsample registered cloud to improve performance
    if points_np.shape[0] > 20000:
        points_np = points_np[::points_np.shape[0] // 20000]

    latest_data["cloud_registered"] = points_np.tobytes()

def path_callback(msg):
    poses = []
    for pose_stamped in msg.poses:
        p = pose_stamped.pose.position
        poses.append({"x": p.x, "y": p.y, "z": p.z})
    latest_data["path"] = poses

def arrival_status_callback(msg):
    latest_data["arrival_status"] = msg.data
    if msg.data == "arrived":
        # Check if there are more waypoints in the queue
        if latest_data["waypoint_queue"]:
            next_point = latest_data["waypoint_queue"].pop(0)
            rospy.loginfo(f"Dispatching next waypoint: {next_point}")
            
            # Publish next point
            pt_msg = PointStamped()
            pt_msg.header.stamp = rospy.Time.now()
            pt_msg.header.frame_id = 'map'
            pt_msg.point.x = next_point['x']
            pt_msg.point.y = next_point['y']
            pt_msg.point.z = next_point['z']
            
            if 'clicked_point_pub' in globals():
                clicked_point_pub.publish(pt_msg)
                # Reset arrival status to indicate we are moving to new point
                latest_data["arrival_status"] = None
                
                # Notify frontend about queue update
    asyncio.create_task(broadcast_queue_update())

async def broadcast_queue_update():
    if 'connected_sids' in globals():
        for sid in list(connected_sids):
             await sio.emit('waypoint_queue', {'queue': latest_data["waypoint_queue"]}, room=sid)

def to_mat44_from_pose(position, orientation):
    q = [orientation["x"], orientation["y"], orientation["z"], orientation["w"]]
    T = quaternion_matrix(q)
    T[0, 3] = position["x"]
    T[1, 3] = position["y"]
    T[2, 3] = position["z"]
    return T

def mat_to_pose(T):
    # Extract translation
    pos = {"x": float(T[0, 3]), "y": float(T[1, 3]), "z": float(T[2, 3])}
    # Extract quaternion from rotation matrix
    # tf.transformations does not provide direct matrix->quat; implement via numpy
    import numpy as _np
    R = T[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = _np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = _np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = _np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = _np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    ori = {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)}
    return pos, ori

def update_submap_boxes():
    try:
        b = latest_data.get("baselink2map")
        o = latest_data.get("odom2map")
        if not b or not o:
            return
        # Build matrices
        Tb = to_mat44_from_pose(b["position"], b["orientation"])  # map -> base_link pose as map frame coords
        To = to_mat44_from_pose(o["position"], o["orientation"])  # map -> odom pose as map frame coords
        import numpy as _np
        # Compute baselink->odom in odom frame: T_odom_baselink = To^{-1} * Tb
        To_inv = _np.linalg.inv(To)
        T_odom_baselink = _np.dot(To_inv, Tb)
        # Map box (history submap) pose is Tb in map frame
        map_pos, map_ori = mat_to_pose(Tb)
        # Scan box (perception submap) pose in map frame: T_map_scan = To * T_odom_baselink_center
        # We already have T_odom_baselink; convert it into map frame by To * T_odom_baselink
        T_map_scan = _np.dot(To, T_odom_baselink)
        scan_pos, scan_ori = mat_to_pose(T_map_scan)
        extent = {"x": 6.0, "y": 6.0, "z": 4.0}
        latest_data["submap_boxes"] = {
            "map_box": {"position": map_pos, "orientation": map_ori, "extent": extent},
            "scan_box": {"position": scan_pos, "orientation": scan_ori, "extent": extent}
        }
    except Exception:
        pass

def baselink2map_callback(msg):
    pose = msg.pose.pose
    position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
    orientation = {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w}
    latest_data["baselink2map"] = {"position": position, "orientation": orientation}
    update_submap_boxes()

def odom2map_callback(msg):
    pose = msg.pose.pose
    position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
    orientation = {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w}
    latest_data["odom2map"] = {"position": position, "orientation": orientation}
    update_submap_boxes()

def ros_thread_entry():
    rospy.init_node('fastapi_visualizer', anonymous=True, disable_signals=True)
    rospy.Subscriber('/map', PointCloud2, global_points_callback)
    rospy.Subscriber('/localization', Odometry, localization_callback)
    rospy.Subscriber('/baselink2map', Odometry, baselink2map_callback)
    rospy.Subscriber('/odom2map', Odometry, odom2map_callback)
    # rospy.Subscriber('/cloud_registered_body', PointCloud2, cur_scan_callback) # Disabled
    rospy.Subscriber('/cloud_registered', PointCloud2, cloud_registered_callback)
    rospy.Subscriber('/pct_path', Path, path_callback)
    rospy.Subscriber('/arrival_status', String, arrival_status_callback)
    rospy.Subscriber('/target_point', Point, target_point_callback) # Subscribe to target point
    globals()['initialpose_pub'] = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    globals()['clicked_point_pub'] = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
    globals()['motion_control_pub'] = rospy.Publisher('/motion_control', Bool, queue_size=1, latch=True)
    rospy.spin()

# Start ROS thread
threading.Thread(target=ros_thread_entry, daemon=True).start()


# --- FastAPI & SocketIO Configuration ---
# Create SocketIO server (Asynchronous)
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
app = FastAPI()
socket_app = socketio.ASGIApp(sio, app)

# Mount static files
# Use absolute path or ensure CWD is correct. Let's use absolute path to be safe.
import os
base_dir = os.path.dirname(os.path.abspath(__file__))
static_dir = os.path.join(base_dir, "../frontend/static")
template_dir = os.path.join(base_dir, "../frontend")

app.mount("/static", StaticFiles(directory=static_dir), name="static")
templates = Jinja2Templates(directory=template_dir)

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@sio.event
async def connect(sid, environ):
    if 'connected_sids' not in globals():
        globals()['connected_sids'] = set()
    if 'client_prefs' not in globals():
        globals()['client_prefs'] = {}
    connected_sids.add(sid)
    client_prefs[sid] = {
        'global_points': True,
        'localization': True,
        # 'cur_scan': True, # Disabled
        'cloud_registered': True,
        'path': True,
        'arrival_status': True
    }
    # if latest_data["global_points"]:
    #    await sio.emit('global_points', latest_data["global_points"], room=sid)
    if latest_data["localization"]:
        await sio.emit('localization', latest_data["localization"], room=sid)
    # if latest_data["cur_scan"]:
    #     await sio.emit('cur_scan', latest_data["cur_scan"], room=sid)
    if latest_data["cloud_registered"]:
        await sio.emit('cloud_registered', latest_data["cloud_registered"], room=sid)
    if latest_data["path"]:
        await sio.emit('path', {'path': latest_data["path"]}, room=sid)
    if latest_data["arrival_status"]:
        await sio.emit('arrival_status', {'status': latest_data["arrival_status"]}, room=sid)
    if latest_data["waypoint_queue"]:
        await sio.emit('waypoint_queue', {'queue': latest_data["waypoint_queue"]}, room=sid)
    if latest_data["target_point"]:
        await sio.emit('target_point', latest_data["target_point"], room=sid)
    if latest_data.get("submap_boxes"):
        await sio.emit('submap_boxes', latest_data["submap_boxes"], room=sid)

@sio.event
async def disconnect(sid):
    if 'connected_sids' in globals():
        connected_sids.discard(sid)
    if 'client_prefs' in globals() and sid in client_prefs:
        client_prefs.pop(sid, None)

# Background task to broadcast updates
async def broadcast_loop():
    while True:
        if 'connected_sids' in globals() and 'client_prefs' in globals():
            for sid in list(connected_sids):
                prefs = client_prefs.get(sid, {})
                if prefs.get('localization') and latest_data["localization"]:
                    await sio.emit('localization', latest_data["localization"], room=sid)
                # if prefs.get('cur_scan') and latest_data["cur_scan"]:
                #     await sio.emit('cur_scan', latest_data["cur_scan"], room=sid)
                if prefs.get('cloud_registered') and latest_data["cloud_registered"]:
                    await sio.emit('cloud_registered', latest_data["cloud_registered"], room=sid)
                if prefs.get('path') and latest_data["path"]:
                    await sio.emit('path', {'path': latest_data["path"]}, room=sid)
                if prefs.get('arrival_status') and latest_data["arrival_status"]:
                     await sio.emit('arrival_status', {'status': latest_data["arrival_status"]}, room=sid)
                
                if latest_data["target_point"]:
                    await sio.emit('target_point', latest_data["target_point"], room=sid)
                if latest_data.get("submap_boxes"):
                    await sio.emit('submap_boxes', latest_data["submap_boxes"], room=sid)

                # Restore global points broadcast but with a check to avoid spamming
                # Frontend will ignore if it already has data
                # if prefs.get('global_points') and latest_data["global_points"]:
                    # Only send occasionally or if we suspect client missed it?
                    # Actually, let's just send it. Frontend has a check "if (globalPointsCloud) return;"
                    # So it's safe to send repeatedly (waste bandwidth but ensures delivery)
                    # To save bandwidth, we can use a counter or timestamp check per client
                    # For now, just uncomment it to fix "not showing" issue.
                    # await sio.emit('global_points', latest_data["global_points"], room=sid)
        await asyncio.sleep(0.05) # Increase loop rate slightly for smoother localization (20Hz)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_loop())

@sio.on('request_global_points')
async def request_global_points(sid):
    # Force clear cached data to ensure fresh read if possible
    # (Though in this architecture, we rely on ROS callback to update latest_data)
    if latest_data["global_points"]:
        await sio.emit('global_points', latest_data["global_points"], room=sid)

@sio.on('set_receive')
async def set_receive(sid, data):
    topic = data.get('topic')
    enabled = bool(data.get('enabled'))
    if 'client_prefs' in globals() and sid in client_prefs and topic in client_prefs[sid]:
        client_prefs[sid][topic] = enabled

@sio.on('set_initialpose')
async def set_initialpose(sid, data):
    try:
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        z = float(data.get('z', 0.0))
        yaw_deg = float(data.get('yaw_deg', 0.0))
        yaw = np.deg2rad(yaw_deg)
        qz = np.sin(yaw/2.0)
        qw = np.cos(yaw/2.0)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # simple covariance
        msg.pose.covariance = [0]*36
        if 'initialpose_pub' in globals():
            initialpose_pub.publish(msg)
    except Exception as e:
        pass

@sio.on('set_clicked_point')
async def set_clicked_point(sid, data):
    try:
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        z = float(data.get('z', 0.0))
        enqueue = bool(data.get('enqueue', False))
        
        if enqueue:
            # Add to queue
            latest_data["waypoint_queue"].append({'x': x, 'y': y, 'z': z})
            await broadcast_queue_update()
        else:
            # Immediate execution
            # We do NOT clear the queue here anymore, allowing "Go Now" to act as an immediate waypoint 
            # followed by the rest of the queue upon arrival.
            # latest_data["waypoint_queue"] = [] # Removed to preserve queue
            # await broadcast_queue_update()
            
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.point.x = x
            msg.point.y = y
            msg.point.z = z
            
            if 'clicked_point_pub' in globals():
                clicked_point_pub.publish(msg)
                # Also reset arrival status when new point is set
                latest_data["arrival_status"] = None
            
    except Exception as e:
        pass

@sio.on('clear_waypoint_queue')
async def clear_waypoint_queue(sid):
    latest_data["waypoint_queue"] = []
    await broadcast_queue_update()

@sio.on('start_queue')
async def start_queue(sid):
    try:
        if latest_data["waypoint_queue"]:
            # Pop first point and execute
            next_point = latest_data["waypoint_queue"].pop(0)
            
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.point.x = next_point['x']
            msg.point.y = next_point['y']
            msg.point.z = next_point['z']
            
            if 'clicked_point_pub' in globals():
                clicked_point_pub.publish(msg)
                latest_data["arrival_status"] = None
                
            await broadcast_queue_update()
    except Exception as e:
        pass

@sio.on('set_motion_control')
async def set_motion_control(sid, data):
    try:
        enabled = bool(data.get('enabled'))
        msg = Bool()
        msg.data = enabled
        if 'motion_control_pub' in globals():
            motion_control_pub.publish(msg)
    except Exception as e:
        pass

if __name__ == "__main__":
    # Run with: uvicorn app_fastapi:socket_app --host 0.0.0.0 --port 5000 --reload
    # But since we use __main__, we can run programmatically
    uvicorn.run(socket_app, host="0.0.0.0", port=5000)
