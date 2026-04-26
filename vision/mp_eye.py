import cv2
import mediapipe as mp
import os
import serial
import time
import threading
import numpy as np
import math

# --- PHYSICAL CONSTANTS (mm) ---
# Your camera is 200mm above the table. 
# The robot base is 100mm above the table.
# This means the camera is 100mm above the robot's "shoulder."
HEIGHT_ABOVE_TARGET = 100.0  # Vertical distance from camera to robot base plane

# --- TUNING ---
ALPHA = 0.15             
EYE_SENSITIVITY_X = 0.6  
EYE_SENSITIVITY_Y = 0.5  

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- SERIAL SETUP ---
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    time.sleep(2)
    print("ESP32 Connected. Robot Base is the target.")
except:
    ser = None
    print("ESP32 not found. Simulation mode.")

# --- CAMERA STREAM ---
class CameraStream:
    def __init__(self, src='tcp://127.0.0.1:8888'):
        self.stream = cv2.VideoCapture(src)
        self.ret, self.frame = self.stream.read()
        self.stopped = False
    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self
    def update(self):
        while not self.stopped:
            ret, frame = self.stream.read()
            if ret: self.ret, self.frame = ret, frame
    def read(self): return self.ret, self.frame
    def stop(self): self.stopped = True

# --- INIT MEDIAPIPE ---
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True)

face_3d_model = np.array([
    (0.0, 0.0, 0.0), (0.0, -330.0, -65.0), (-225.0, 170.0, -135.0),
    (225.0, 170.0, -135.0), (-150.0, -150.0, -125.0), (150.0, -150.0, -125.0)
], dtype=np.float64)

cam = CameraStream().start()
smooth_x, smooth_z = 0.0, 0.0

# These will store the angles when looking at the ROBOT BASE
calib_head_yaw, calib_head_pitch = 0.0, 0.0
calib_eye_x, calib_eye_y = 0.0, 0.0

print("READY: Stare at the ROBOT ARM BASE and press SPACE.")

while True:
    ret, frame = cam.read()
    if not ret: continue

    frame = cv2.flip(cv2.rotate(frame, cv2.ROTATE_180), 1)
    h, w, _ = frame.shape
    small_rgb = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2RGB)
    results = face_mesh.process(small_rgb)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        # 1. HEAD POSE
        img_pts = np.array([
            (landmarks[1].x * w, landmarks[1].y * h), (landmarks[152].x * w, landmarks[152].y * h),
            (landmarks[33].x * w, landmarks[33].y * h), (landmarks[263].x * w, landmarks[263].y * h),
            (landmarks[61].x * w, landmarks[61].y * h), (landmarks[291].x * w, landmarks[291].y * h)
        ], dtype=np.float64)
        
        cam_matrix = np.array([[w, 0, w/2], [0, w, h/2], [0, 0, 1]], dtype=np.float64)
        _, rvec, _ = cv2.solvePnP(face_3d_model, img_pts, cam_matrix, np.zeros((4,1)))
        rmat, _ = cv2.Rodrigues(rvec)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
        head_pitch, head_yaw = angles[0], angles[1]

        # 2. STABLE GAZE LOGIC (Droop-Proof)
        def get_stable_gaze(iris_idx, x_ref_idx, y_ref_idx, l_corner_idx, r_corner_idx):
            iris = landmarks[iris_idx]
            scale = abs(landmarks[r_corner_idx].x - landmarks[l_corner_idx].x)
            rel_x = (iris.x - landmarks[x_ref_idx].x) / scale
            rel_y = (iris.y - landmarks[y_ref_idx].y) / scale
            return rel_x, rel_y

        rx, ry = get_stable_gaze(468, 159, 247, 133, 33)
        lx, ly = get_stable_gaze(473, 386, 467, 362, 263)
        
        curr_eye_x = (lx + rx) / 2.0
        curr_eye_y = (ly + ry) / 2.0

        # 3. CALCULATE MOVEMENT FROM ROBOT BASE
        # We find the difference in angle between "Now" and "When I looked at the base"
        delta_yaw_rad = math.radians(head_yaw - calib_head_yaw) + (curr_eye_x - calib_eye_x) * EYE_SENSITIVITY_X
        delta_pitch_rad = math.radians(head_pitch - calib_head_pitch) - (curr_eye_y - calib_eye_y) * EYE_SENSITIVITY_Y

        # 4. TRIG (Ray Casting from Robot Base)
        # If delta is 0, tan(0) is 0, so target is 0,0 (The Robot Base!)
        target_x = HEIGHT_ABOVE_TARGET * math.tan(delta_yaw_rad)
        target_z = HEIGHT_ABOVE_TARGET * math.tan(delta_pitch_rad)

        # 5. SMOOTHING
        smooth_x = (ALPHA * target_x) + (1 - ALPHA) * smooth_x
        smooth_z = (ALPHA * target_z) + (1 - ALPHA) * smooth_z

        # 6. SEND TO ESP32
        if ser:
            # We are sending coordinates relative to the base. 
            # 0,0,0 is the base itself.
            data = f"{smooth_x:.1f},0.0,{smooth_z:.1f}\n"
            ser.write(data.encode())

    if not is_headless:
        cv2.imshow('HackABull Robot-Center', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            # Capture the current "Looking at Robot Base" pose
            calib_head_yaw, calib_head_pitch = head_yaw, head_pitch
            calib_eye_x, calib_eye_y = curr_eye_x, curr_eye_y
            print(">>> ORIGIN SET: Robot Base is now 0,0,0 <<<")
        elif key == ord('q'): break

cam.stop()
if ser: ser.close()
cv2.destroyAllWindows()