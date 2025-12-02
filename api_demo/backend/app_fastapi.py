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

# --- ROS Configuration ---
# Global storage for latest data
latest_data = {
    "global_points": [],
    "localization": None,
    "cur_scan": [],
    "path": []
}

import numpy as np
# ROS Callbacks
def global_points_callback(msg):
    # Directly access binary data from PointCloud2
    # This is raw binary data (typically float32 x, y, z and maybe intensity/rgb)
    # To be safe and efficient, we extract x, y, z using a generator but pack into a binary blob immediately
    # Or better, we use numpy_msg from ros_numpy if available, but standard pc2 is safer for dependencies
    
    # Use pc2.read_points to get generator, but instead of creating dicts, create flat list or numpy array
    # Optimization: Use read_points_list if available or just list()
    # Even faster: struct unpacking if structure is known.
    # For compatibility, let's stick to read_points but flatten to float array immediately.
    
    points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    # Convert to numpy array directly
    points_np = np.array(list(points_gen), dtype=np.float32)
    
    # Sampling if too large
    if points_np.shape[0] > 50000:
         # Simple strided sampling
         points_np = points_np[::points_np.shape[0] // 50000]
         
    # Store as bytes for efficient transmission
    latest_data["global_points"] = points_np.tobytes()

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

def cur_scan_callback(msg):
    points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_np = np.array(list(points_gen), dtype=np.float32)
    
    if points_np.shape[0] > 20000:
         points_np = points_np[::points_np.shape[0] // 20000]
         
    latest_data["cur_scan"] = points_np.tobytes()

def path_callback(msg):
    poses = []
    for pose_stamped in msg.poses:
        p = pose_stamped.pose.position
        poses.append({"x": p.x, "y": p.y, "z": p.z})
    latest_data["path"] = poses

def ros_thread_entry():
    rospy.init_node('fastapi_visualizer', anonymous=True, disable_signals=True)
    rospy.Subscriber('/map', PointCloud2, global_points_callback)
    rospy.Subscriber('/localization', Odometry, localization_callback)
    rospy.Subscriber('/cur_scan_in_map', PointCloud2, cur_scan_callback)
    rospy.Subscriber('/pct_path', Path, path_callback)
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
    print(f"Client connected: {sid}")
    # Send initial data
    if latest_data["global_points"]:
        await sio.emit('global_points', latest_data["global_points"], room=sid)
    if latest_data["localization"]:
        await sio.emit('localization', latest_data["localization"], room=sid)
    if latest_data["cur_scan"]:
        await sio.emit('cur_scan', latest_data["cur_scan"], room=sid)
    if latest_data["path"]:
        await sio.emit('path', {'path': latest_data["path"]}, room=sid)

@sio.event
async def disconnect(sid):
    print(f"Client disconnected: {sid}")

# Background task to broadcast updates
async def broadcast_loop():
    while True:
        # In a real high-perf scenario, we'd use events/queues. 
        # Polling global vars is simple for this demo.
        if latest_data["localization"]:
            await sio.emit('localization', latest_data["localization"])
        
        if latest_data["cur_scan"]:
             await sio.emit('cur_scan', latest_data["cur_scan"])
             
        if latest_data["path"]:
            await sio.emit('path', {'path': latest_data["path"]})
            
        # Global points usually static or slow changing, maybe send less frequently?
        # For now, let's rely on on-connect or occasional updates if changed.
        # Assuming global points don't change rapidly for this demo.
        
        await asyncio.sleep(0.1) # 10Hz update rate

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_loop())

if __name__ == "__main__":
    # Run with: uvicorn app_fastapi:socket_app --host 0.0.0.0 --port 5000 --reload
    # But since we use __main__, we can run programmatically
    uvicorn.run(socket_app, host="0.0.0.0", port=5000)
