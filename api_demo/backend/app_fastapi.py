import uvicorn
import socketio
import threading
import asyncio
import rospy
import sensor_msgs.point_cloud2 as pc2
from fastapi import FastAPI, Request, Response
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_matrix, quaternion_multiply
import os
import time
import urllib.request

from pydantic import BaseModel


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
    "map_to_odom": None
}

import numpy as np
mode_lock = threading.Lock()
mode_state = {"mode": "idle", "mapping": False, "navigation": False}
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

def occupancy_inflate_callback(msg):
    points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_np = np.array(list(points_gen), dtype=np.float32)
    if points_np.shape[0] > 100000:
        points_np = points_np[::points_np.shape[0] // 100000]
    latest_data["occupancy_inflate"] = points_np.tobytes()

def local_traj_callback(msg: Marker):
    pts = []
    for p in msg.points:
        pts.append({"x": p.x, "y": p.y, "z": p.z})
    latest_data["local_traj"] = {"points": pts}

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

def map_to_odom_callback(msg):
    pose = msg.pose.pose
    position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
    orientation = {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w}
    latest_data["map_to_odom"] = {"position": position, "orientation": orientation}

def ros_thread_entry():
    rospy.init_node('fastapi_visualizer', anonymous=True, disable_signals=True)
    rospy.Subscriber('/map', PointCloud2, global_points_callback)
    rospy.Subscriber('/localization', Odometry, localization_callback)
    rospy.Subscriber('/map_to_odom', Odometry, map_to_odom_callback)
    # rospy.Subscriber('/cloud_registered_body', PointCloud2, cur_scan_callback) # Disabled
    rospy.Subscriber('/cloud_registered', PointCloud2, cloud_registered_callback)
    rospy.Subscriber('/sdf_map/occupancy_inflate', PointCloud2, occupancy_inflate_callback)
    rospy.Subscriber('/sdf_map/occupancy', PointCloud2, occupancy_inflate_callback)
    rospy.Subscriber('/planning_vis/trajectory', Marker, local_traj_callback)
    rospy.Subscriber('/pct_path', Path, path_callback)
    rospy.Subscriber('/arrival_status', String, arrival_status_callback)
    rospy.Subscriber('/target_point', Point, target_point_callback) # Subscribe to target point
    globals()['initialpose_pub'] = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    globals()['clicked_point_pub'] = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
    globals()['motion_control_pub'] = rospy.Publisher('/motion_control', Bool, queue_size=1, latch=True)
    globals()['build_launch_pub'] = rospy.Publisher('build_start_stop_launch', Bool, queue_size=1, latch=True)
    globals()['nav_launch_pub'] = rospy.Publisher('nav_start_stop_launch', Bool, queue_size=1, latch=True)
    rospy.spin()

# Start ROS thread
threading.Thread(target=ros_thread_entry, daemon=True).start()


# --- FastAPI & SocketIO Configuration ---
# Create SocketIO server (Asynchronous)
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
app = FastAPI()
socket_app = socketio.ASGIApp(sio, app)

base_dir = os.path.dirname(os.path.abspath(__file__))
static_dir = os.path.join(base_dir, "../frontend/static")
template_dir = os.path.join(base_dir, "../frontend")

app.mount("/static", StaticFiles(directory=static_dir), name="static")
templates = Jinja2Templates(directory=template_dir)

def _publish_toggle(pub_name: str, enabled: bool):
    pub = globals().get(pub_name)
    if pub is None:
        return False
    msg = Bool()
    msg.data = bool(enabled)
    try:
        pub.publish(msg)
        return True
    except Exception:
        return False

def _set_mode(mode: str, enabled: bool):
    mode = (mode or "").strip().lower()
    enabled = bool(enabled)
    if mode not in ("mapping", "navigation"):
        return {"ok": False, "error": "invalid_mode"}
    with mode_lock:
        if mode == "mapping":
            if enabled:
                _publish_toggle("nav_launch_pub", False)
                mode_state["navigation"] = False
            ok = _publish_toggle("build_launch_pub", enabled)
            mode_state["mapping"] = enabled
        else:
            if enabled:
                _publish_toggle("build_launch_pub", False)
                mode_state["mapping"] = False
            ok = _publish_toggle("nav_launch_pub", enabled)
            mode_state["navigation"] = enabled
        if mode_state["mapping"]:
            mode_state["mode"] = "mapping"
        elif mode_state["navigation"]:
            mode_state["mode"] = "navigation"
        else:
            mode_state["mode"] = "idle"
    return {"ok": bool(ok), "state": dict(mode_state)}

class ToggleRequest(BaseModel):
    enabled: bool = True

@app.get("/api/mode")
async def get_mode():
    return dict(mode_state)

@app.post("/api/mapping")
async def set_mapping(req: ToggleRequest):
    return _set_mode("mapping", req.enabled)

@app.post("/api/navigation")
async def set_navigation(req: ToggleRequest):
    return _set_mode("navigation", req.enabled)

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

def generate_frames():
    base_url = os.environ.get('SAFEHAT_BASE_URL', 'http://127.0.0.1:8001').rstrip('/')
    frame_url = f"{base_url}/frame.jpg"
    while True:
        try:
            with urllib.request.urlopen(frame_url, timeout=2.0) as resp:
                if getattr(resp, 'status', 200) != 200:
                    time.sleep(0.1)
                    continue
                frame_bytes = resp.read()
                if not frame_bytes:
                    time.sleep(0.05)
                    continue
        except Exception:
            time.sleep(0.1)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

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
    if latest_data.get("map_to_odom"):
        await sio.emit('map_to_odom', latest_data["map_to_odom"], room=sid)
    if latest_data.get("occupancy_inflate"):
        await sio.emit('occupancy_inflate', latest_data["occupancy_inflate"], room=sid)
    if latest_data.get("local_traj"):
        await sio.emit('local_traj', latest_data["local_traj"], room=sid)

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
                if latest_data.get("map_to_odom"):
                    await sio.emit('map_to_odom', latest_data["map_to_odom"], room=sid)
                if latest_data.get("occupancy_inflate"):
                    await sio.emit('occupancy_inflate', latest_data["occupancy_inflate"], room=sid)
                if latest_data.get("local_traj"):
                    await sio.emit('local_traj', latest_data["local_traj"], room=sid)

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

@sio.on('set_mapping')
async def sio_set_mapping(sid, data):
    enabled = True
    if isinstance(data, dict):
        enabled = bool(data.get("enabled", True))
    return _set_mode("mapping", enabled)

@sio.on('set_navigation')
async def sio_set_navigation(sid, data):
    enabled = True
    if isinstance(data, dict):
        enabled = bool(data.get("enabled", True))
    return _set_mode("navigation", enabled)

@sio.on('get_mode')
async def sio_get_mode(sid):
    return dict(mode_state)

if __name__ == "__main__":
    # Run with: uvicorn app_fastapi:socket_app --host 0.0.0.0 --port 5000 --reload
    # But since we use __main__, we can run programmatically
    uvicorn.run(socket_app, host="0.0.0.0", port=5000)
