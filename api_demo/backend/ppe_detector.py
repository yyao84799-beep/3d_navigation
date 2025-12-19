
import cv2
import cvzone
import math
import threading
import time
from ultralytics import YOLO
import numpy as np

class PPEDetector:
    def __init__(self, model_path="ppe.pt", camera_index=3):
        self.model = YOLO(model_path)
        self.camera_index = camera_index
        self.cap = None
        self.running = False
        self.latest_frame = None
        self.lock = threading.Lock()
        
        self.classNames = ['Excavator', 'Gloves', 'Hardhat', 'Ladder', 'Mask', 'NO-Hardhat', 'NO-Mask',
                      'NO-Safety Vest', 'Person', 'SUV', 'Safety Cone', 'Safety Vest', 'bus',
                      'dump truck', 'fire hydrant', 'machinery', 'mini-van', 'sedan', 'semi',
                      'trailer', 'truck and trailer', 'truck', 'van', 'vehicle', 'wheel loader']
        self.allowed_classes = {'Hardhat', 'Safety Vest', 'NO-Hardhat', 'NO-Safety Vest'}

    def start(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            print(f"Error opening video source {self.camera_index}")
            return False
        self.running = True
        threading.Thread(target=self._process_loop, daemon=True).start()
        return True

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def _process_loop(self):
        while self.running and self.cap.isOpened():
            success, img = self.cap.read()
            if not success:
                # Try to reconnect or just wait
                time.sleep(0.1)
                continue

            results = self.model(img, stream=True, classes=[2, 5, 11, 7], verbose=False)

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    w, h = x2 - x1, y2 - y1
                    conf = math.ceil((box.conf[0] * 100)) / 100
                    cls = int(box.cls[0])
                    
                    if cls < len(self.classNames):
                        currentClass = self.classNames[cls]
                    else:
                        continue

                    if currentClass not in self.allowed_classes:
                        continue

                    if conf > 0.5:
                        if currentClass in ['NO-Hardhat', 'NO-Safety Vest']:
                            myColor = (0, 0, 255)
                        elif currentClass in ['Hardhat', 'Safety Vest']:
                            myColor = (0, 255, 0)
                        else:
                            myColor = (255, 0, 0)

                        cvzone.putTextRect(img, f'{currentClass} {conf}',
                                           (max(0, x1), max(35, y1)), scale=1, thickness=1,
                                           colorB=myColor, colorT=(255, 255, 255), colorR=myColor, offset=5)
                        cv2.rectangle(img, (x1, y1), (x2, y2), myColor, 3)

            with self.lock:
                self.latest_frame = img
            
            # Limit frame rate slightly to save CPU if needed
            time.sleep(0.01)

