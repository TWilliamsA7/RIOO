import cv2
import mediapipe as mp
import os
import serial
import time
import threading
import numpy as np
import math

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# ==========================================
# PHYSICAL REACH LIMITS & TUNING
# ==========================================
MAX_REACH_X = 150.0  # mm
MAX_REACH_Z = 150.0  # mm
FIXED_Y = -100.0     # mm

# Head Compensation (Calibrate these based on your "Freeze Test")
HEAD_YAW_WEIGHT = 0.5   
HEAD_PITCH_WEIGHT = 0.5 
# ==========================================

# --- Fixed Camera Class ---
class CameraStream:
    def __init__(self, src='tcp://127.0.0.1:8888'):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self.ret, self.frame = self.stream.read()
        self.stopped = False
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.update, args=(), daemon=True)
        self.thread.start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.stream.read()
            if ret:
                self.ret = ret
                self.frame = frame
            else:
                time.sleep(0.01) 
    def read(self): return self.ret, self.frame
    def stop(self):
        self.stopped = True
        if self.thread is not None: self.thread.join(timeout=1.0)
        self.stream.release()

# --- Serial Setup ---
ESP_PORT = '/dev/ttyAMA0' 
try:
    ser = serial.Serial(ESP_PORT, 115200, timeout=1)
    time.sleep(2)
except: ser = None

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.4, min_tracking_confidence=0.4)

face_3d_model = np.array([(0,0,0), (0,-330,-65), (-225,170,-135), (225,170,-135), (-150,-150,-125), (150,-150,-125)], dtype=np.float64)

# Calibration & Smoothing Variables
offset_x, offset_y = 0.0, 0.0
smooth_x, smooth_y = 0.0, 0.0

cam = CameraStream().start()
time.sleep(1)

while True:
    ret, frame = cam.read()
    if not ret or frame is None: continue

    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape
    rgb_frame = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        # 1. SOLVE PNP
        face_2d = np.array([(landmarks[i].x*w, landmarks[i].y*h) for i in [1, 152, 33, 263, 61, 291]], dtype=np.float64)
        f = 1.0 * w
        mat = np.array([[f, 0, w/2], [0, f, h/2], [0, 0, 1]], dtype=np.float64)
        success, rvec, tvec = cv2.solvePnP(face_3d_model, face_2d, mat, np.zeros((4,1)), flags=cv2.SOLVEPNP_SQPNP)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(cv2.Rodrigues(rvec)[0])
        yaw, pitch = angles[1], angles[0]

        # 2. NORMALIZED IRIS (-1 to 1)
        iris_l, iris_r = landmarks[473], landmarks[468]
        # x: (val - 0.5) * 2 -> Left is -1, Right is 1
        raw_x = ((iris_l.x + iris_r.x) / 2.0 - 0.5) * 2.0
        # y: Inverted! -> -(val - 0.5) * 2 -> Down is -1, Up is 1
        raw_y = -((iris_l.y + iris_r.y) / 2.0 - 0.5) * 2.0

        # 3. COMBINE WITH HEAD POSE
        combined_x = raw_x - (yaw * 0.001 * HEAD_YAW_WEIGHT)
        combined_y = raw_y - (pitch * 0.001 * HEAD_PITCH_WEIGHT)

        # 4. INDEPENDENT DEADZONES
        delta_x = (combined_x - offset_x) - smooth_x
        delta_y = (combined_y - offset_y) - smooth_y

        # Deadzone: Ignore jitter less than 0.05 units
        if abs(delta_x) > 0.05: smooth_x += delta_x * 0.3
        if abs(delta_y) > 0.05: smooth_y += delta_y * 0.3

        # Scale to MM
        target_x = smooth_x * MAX_REACH_X
        target_z = smooth_y * MAX_REACH_Z
        target_y = FIXED_Y

        # 5. COMMUNICATION
        if ser:
            if abs(target_x) < 200 and abs(target_z) < 200:
                ser.write(f"{target_x:.1f},{target_y:.1f},{target_z:.1f}\n".encode())

        # VISUALS
        if success: cv2.drawFrameAxes(frame, mat, np.zeros((4,1)), rvec, tvec, 150, 2)
        # Normalized circle drawing
        cv2.circle(frame, (int(((raw_x/2.0)+0.5)*w), int((( (-raw_y)/2.0)+0.5)*h)), 5, (0,255,0), -1)

    if not is_headless:
        cv2.imshow('Tracker', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord(' '): 
            offset_x, offset_y = combined_x, combined_y
            smooth_x, smooth_y = 0.0, 0.0
            print(">>> CALIBRATED (Pure Offset) <<<")

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()