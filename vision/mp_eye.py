import cv2
import mediapipe as mp
import os
import serial
import time
import threading
import numpy as np
import math

# --- PHYSICAL CONSTANTS (mm) ---
CAM_HEIGHT_ABOVE_TABLE = 200.0
ROBOT_X_OFFSET = 100.0   # Robot is 100mm to the right of camera
ROBOT_Z_OFFSET = 0.0     # Robot is in line with camera
ROBOT_BASE_Y = -100.0    # Robot base is 100mm above table (100mm below cam)

# --- TUNING ---
ALPHA = 0.15             # Smoothing factor
EYE_SENS_X = 2.5         # CRANKED UP: 5cm travel means this needs to be high
EYE_SENS_Y = 2.0         

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- SERIAL SETUP ---
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    time.sleep(2)
    print("ESP32 Connected.")
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

# 3D Model for Pose Estimation
face_3d_model = np.array([
    (0.0, 0.0, 0.0), (0.0, -330.0, -65.0), (-225.0, 170.0, -135.0),
    (225.0, 170.0, -135.0), (-150.0, -150.0, -125.0), (150.0, -150.0, -125.0)
], dtype=np.float64)

cam = CameraStream().start()
smooth_x, smooth_z = 0.0, 0.0

# Calibration Deltas
calib_total_yaw, calib_total_pitch = 0.0, 0.0

print("\n" + "="*50)
print("ACTION: STARE AT THE ROBOT BASE AND HIT SPACE")
print("="*50 + "\n")

while True:
    ret, frame = cam.read()
    if not ret: continue

    frame = cv2.flip(cv2.rotate(frame, cv2.ROTATE_180), 1)
    h, w, _ = frame.shape
    small_rgb = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2RGB)
    results = face_mesh.process(small_rgb)

    if results.multi_face_landmarks:
        mesh = results.multi_face_landmarks[0].landmark
        
        # --- DRAW VISUAL DOTS ---
        # Blue: X-Ref, Green: Y-Ref (Orbital bone), Red: Irises
        for idx, color in [(159, (255,0,0)), (386, (255,0,0)), (247, (0,255,0)), 
                           (467, (0,255,0)), (468, (0,0,255)), (473, (0,0,255))]:
            cv2.circle(frame, (int(mesh[idx].x*w), int(mesh[idx].y*h)), 2, color, -1)

        # 1. HEAD POSE (SolvePnP)
        img_pts = np.array([
            (mesh[1].x*w, mesh[1].y*h), (mesh[152].x*w, mesh[152].y*h),
            (mesh[33].x*w, mesh[33].y*h), (mesh[263].x*w, mesh[263].y*h),
            (mesh[61].x*w, mesh[61].y*h), (mesh[291].x*w, mesh[291].y*h)
        ], dtype=np.float64)
        cam_mtx = np.array([[w, 0, w/2], [0, w, h/2], [0, 0, 1]], dtype=np.float64)
        _, rvec, _ = cv2.solvePnP(face_3d_model, img_pts, cam_mtx, np.zeros((4,1)))
        rmat, _ = cv2.Rodrigues(rvec)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
        hy, hp = angles[1], angles[0]

        # 2. DROOP-PROOF EYE LOGIC
        def get_gaze_delta(iris_idx, x_ref_idx, y_ref_idx, l_corner, r_corner):
            # Scale against eye width
            scale = abs(mesh[r_corner].x - mesh[l_corner].x)
            dx = (mesh[iris_idx].x - mesh[x_ref_idx].x) / scale
            dy = (mesh[iris_idx].y - mesh[y_ref_idx].y) / scale
            return dx, dy

        rx, ry = get_gaze_delta(468, 159, 247, 133, 33)
        lx, ly = get_gaze_delta(473, 386, 467, 362, 263)
        
        # Combine Eye + Head into a single "Ray Angle"
        total_yaw = math.radians(hy) + ((lx + rx) / 2.0) * EYE_SENS_X
        total_pitch = math.radians(hp) - ((ly + ry) / 2.0) * EYE_SENS_Y

        # 3. THE "ZEROING" MATH
        # This makes current gaze angle relative to the angle when you hit Space
        delta_yaw = total_yaw - calib_total_yaw
        delta_pitch = total_pitch - calib_total_pitch

        # 4. TRIGONOMETRY (X and Z travel on the table)
        # Using the actual 200mm height you measured
        raw_x = CAM_HEIGHT_ABOVE_TABLE * math.tan(delta_yaw)
        raw_z = CAM_HEIGHT_ABOVE_TABLE * math.tan(delta_pitch)

        # 5. ROBOT ABSOLUTE TRANSLATION
        # When delta is 0 (looking at base), target_x should be 0 relative to robot
        target_x = raw_x 
        target_z = raw_z

        # 6. SMOOTHING
        smooth_x = (ALPHA * target_x) + (1 - ALPHA) * smooth_x
        smooth_z = (ALPHA * target_z) + (1 - ALPHA) * smooth_z

        # --- TERMINAL OUTPUT ---
        print(f"X: {smooth_x:6.1f} | Z: {smooth_z:6.1f} | HEAD: {hy:5.1f}", end='\r')

        # 7. SEND TO ESP32
        if ser:
            # We send coordinates where 0,0,0 is the Robot Base
            data = f"{smooth_x:.1f},{ROBOT_BASE_Y:.1f},{smooth_z:.1f}\n"
            ser.write(data.encode())

    if not is_headless:
        cv2.imshow('HackABull Diagnostic', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            calib_total_yaw = total_yaw
            calib_total_pitch = total_pitch
            print("\n>>> ZEROED: Robot Base is now 0,0. <<<\n")
        elif key == ord('q'): break

cam.stop()
if ser: ser.close()
cv2.destroyAllWindows()