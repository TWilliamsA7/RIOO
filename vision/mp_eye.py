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
ROBOT_X_OFFSET = 100.0   
ROBOT_Y_OFFSET = 100.0   
ROBOT_Z_OFFSET = 0.0     
TABLE_Y = 0.0            

# --- TUNING ---
ALPHA = 0.15             # Smoothing factor
EYE_SENSITIVITY_X = 0.6  # Adjust if the arm moves too much/little
EYE_SENSITIVITY_Y = 0.5  

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- SERIAL SETUP ---
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    time.sleep(2)
    print("ESP32 Connected. Let's go!")
except:
    ser = None
    print("ESP32 not found. Simulation mode active.")

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
    (0.0, 0.0, 0.0),            # Nose tip
    (0.0, -330.0, -65.0),       # Chin
    (-225.0, 170.0, -135.0),    # Left eye corner
    (225.0, 170.0, -135.0),     # Right eye corner
    (-150.0, -150.0, -125.0),   # Left mouth
    (150.0, -150.0, -125.0)     # Right mouth
], dtype=np.float64)

cam = CameraStream().start()
smooth_x, smooth_z = 0.0, 0.0
calib_head_yaw, calib_head_pitch = 0.0, 0.0
calib_eye_x, calib_eye_y = 0.0, 0.0 # Eye-specific calibration

print("Stable Gaze Active. Stare at Camera and press SPACE.")

while True:
    ret, frame = cam.read()
    if not ret: continue

    frame = cv2.flip(cv2.rotate(frame, cv2.ROTATE_180), 1)
    h, w, _ = frame.shape
    small_rgb = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2RGB)
    results = face_mesh.process(small_rgb)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        # 1. HEAD POSE (SolvePnP)
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

        # 2. CUSTOM STABLE GAZE LOGIC
        # We use bone-stable landmarks (247, 467) for Y and eyelid-midpoints (159, 386) for X
        def get_stable_gaze(iris_idx, x_ref_idx, y_ref_idx, l_corner_idx, r_corner_idx):
            iris = landmarks[iris_idx]
            anchor_x = landmarks[x_ref_idx].x
            anchor_y = landmarks[y_ref_idx].y
            
            # Use distance between corners as the scale (the "box" size)
            scale = abs(landmarks[r_corner_idx].x - landmarks[l_corner_idx].x)
            
            # Calculate raw displacement from the specific landmarks
            rel_x = (iris.x - anchor_x) / scale
            rel_y = (iris.y - anchor_y) / scale
            
            return rel_x, rel_y

        # Right Eye: iris(468), x_ref(159), y_ref(247), corners(133, 33)
        rx, ry = get_stable_gaze(468, 159, 247, 133, 33)
        # Left Eye: iris(473), x_ref(386), y_ref(467), corners(362, 263)
        lx, ly = get_stable_gaze(473, 386, 467, 362, 263)
        
        # Average eyes and subtract eye-calibration
        eye_x = (((lx + rx) / 2.0) - calib_eye_x) * EYE_SENSITIVITY_X
        eye_y = (((ly + ry) / 2.0) - calib_eye_y) * EYE_SENSITIVITY_Y

        # 3. COORDINATE CALCULATION (Angles to Radians)
        total_yaw_rad = math.radians(head_yaw - calib_head_yaw) + eye_x
        total_pitch_rad = math.radians(head_pitch - calib_head_pitch) - eye_y

        # 4. TRIGONOMETRY (Ray Casting)
        # Table hit = Height * tan(Angle)
        table_x = CAM_HEIGHT_ABOVE_TABLE * math.tan(total_yaw_rad)
        table_z = CAM_HEIGHT_ABOVE_TABLE * math.tan(total_pitch_rad)

        # 5. ROBOT POSITIONING
        # Shifts table hit relative to robot base
        rob_x = table_x - ROBOT_X_OFFSET
        rob_z = table_z - ROBOT_Z_OFFSET
        # rob_y is fixed at the table surface level relative to robot base height
        rob_y = -100.0 

        # 6. SMOOTHING
        smooth_x = (ALPHA * rob_x) + (1 - ALPHA) * smooth_x
        smooth_z = (ALPHA * rob_z) + (1 - ALPHA) * smooth_z

        print(f"X: {smooth_x:6.1f} | Z: {smooth_z:6.1f} | HeadYaw: {head_yaw:5.1f}", end='\r')

        # 7. SEND TO ESP32
        if ser:
            data = f"{smooth_x:.1f},{rob_y:.1f},{smooth_z:.1f}\n"
            ser.write(data.encode())

    # Visual Debugging & Calibration
    if not is_headless:
        cv2.imshow('HackABull Stable', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            # Capture current state as "Zero"
            calib_head_yaw, calib_head_pitch = head_yaw, head_pitch
            calib_eye_x = (lx + rx) / 2.0
            calib_eye_y = (ly + ry) / 2.0
            print(">>> Calibrated! Looking straight and bone-stable. <<<")
        elif key == ord('q'): break

cam.stop()
if ser: ser.close()
cv2.destroyAllWindows()