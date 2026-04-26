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
# PHYSICAL REACH LIMITS & WEIGHTS
# ==========================================
MAX_REACH_X = 150.0  # 150mm left or right
MAX_REACH_Z = 150.0  # 150mm forward or backward
FIXED_Y = -100.0     # The table is 100mm below the robot base

# HEAD POSE WEIGHTS:
# Do the Freeze Test! If you turn your head left, and the arm snaps left,
# flip the minus (-) to a plus (+) in the "MERGING" section below.
HEAD_YAW_WEIGHT = 0.02   
HEAD_PITCH_WEIGHT = 0.02 
# ==========================================

# --- Fixed Camera Class (Thread Safe) ---
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

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.stream.release()
# ------------------------------------------

# --- Serial Setup (AMA0) ---
ESP_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 115200
try:
    ser = serial.Serial(ESP_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Connected to ESP32 on {ESP_PORT}!")
except Exception as e:
    print(f"WARNING: Could not connect to ESP32. Running vision only.")
    ser = None
# --------------------

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True, 
    min_detection_confidence=0.4,
    min_tracking_confidence=0.4
)

# 3D Model Points of a Generic Human Face (For solvePnP)
face_3d_model = np.array([
    (0.0, 0.0, 0.0),             # Nose tip
    (0.0, -330.0, -65.0),        # Chin
    (-225.0, 170.0, -135.0),     # Left eye left corner
    (225.0, 170.0, -135.0),      # Right eye right corner
    (-150.0, -150.0, -125.0),    # Left Mouth corner
    (150.0, -150.0, -125.0)      # Right mouth corner
], dtype=np.float64)

print("Connecting to fast camera stream...")
cam = CameraStream().start()
time.sleep(1) 

if not cam.ret:
    print("Failed to open stream.")
    cam.stop()
    exit()

print("Stream connected! Tracking gaze and 6DoF head pose...")

# Variables
offset_x = 0.0
offset_y = 0.0
smooth_x = 0.0
smooth_y = 0.0

while True:
    ret, frame = cam.read()
    if not ret or frame is None: 
        continue

    # APPLY ORIENTATION FIXES
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = cv2.flip(frame, 1)

    h, w, _ = frame.shape
    small_frame = cv2.resize(frame, (320, 240))
    rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
    
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        # ==========================================
        # 1. SOLVE PNP (HEAD POSE ESTIMATION)
        # ==========================================
        face_2d_image = np.array([
            (landmarks[1].x * w, landmarks[1].y * h),     
            (landmarks[152].x * w, landmarks[152].y * h), 
            (landmarks[33].x * w, landmarks[33].y * h),   
            (landmarks[263].x * w, landmarks[263].y * h), 
            (landmarks[61].x * w, landmarks[61].y * h),   
            (landmarks[291].x * w, landmarks[291].y * h)  
        ], dtype=np.float64)

        focal_length = 1.0 * w
        cam_matrix = np.array([[focal_length, 0, w / 2],
                               [0, focal_length, h / 2],
                               [0, 0, 1]], dtype=np.float64)
        dist_matrix = np.zeros((4, 1), dtype=np.float64)

        # Swapped to SQPNP for ultra-stable, non-drifting head rotation
        success, rvec, tvec = cv2.solvePnP(face_3d_model, face_2d_image, cam_matrix, dist_matrix, flags=cv2.SOLVEPNP_SQPNP)
        rmat, _ = cv2.Rodrigues(rvec)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)

        head_pitch = angles[0] 
        head_yaw = angles[1]   
        head_roll = angles[2]  

        # ==========================================
        # 2. DUAL EYE TRACKING (AVERAGED)
        # ==========================================
        def get_gaze(iris_idx, inner_idx, outer_idx, top_idx, bottom_idx):
            iris = landmarks[iris_idx]
            inner = landmarks[inner_idx]
            outer = landmarks[outer_idx]
            top = landmarks[top_idx]
            bottom = landmarks[bottom_idx]
            
            eye_w = abs(outer.x - inner.x)
            eye_h = abs(bottom.y - top.y)
            
            rx = (iris.x - inner.x) / eye_w if eye_w != 0 else 0.5
            ry = (iris.y - top.y) / eye_h if eye_h != 0 else 0.5
            return (rx * 2.0) - 1.0, -((ry * 2.0) - 1.0)

        lx, ly = get_gaze(473, 362, 263, 467, 374)
        rx, ry = get_gaze(468, 133, 33, 247, 145)

        raw_eye_x = (lx + rx) / 2.0
        raw_eye_y = (ly + ry) / 2.0

        # ==========================================
        # 3. MERGING HEAD POSE AND GAZE
        # ==========================================
        roll_rad = math.radians(head_roll)
        derolled_x = (raw_eye_x * math.cos(roll_rad)) - (raw_eye_y * math.sin(roll_rad))
        derolled_y = (raw_eye_x * math.sin(roll_rad)) + (raw_eye_y * math.cos(roll_rad))

        # IF THE FREEZE TEST FAILS, CHANGE THE MINUS (-) TO A PLUS (+) HERE:
        combined_x = derolled_x - (head_yaw * HEAD_YAW_WEIGHT)
        combined_y = derolled_y - (head_pitch * HEAD_PITCH_WEIGHT)

        # ==========================================
        # 4. DYNAMIC SHOCK ABSORBER (ANTI-DRIFT)
        # ==========================================
        delta_x = combined_x - smooth_x
        delta_y = combined_y - smooth_y
        distance = math.hypot(delta_x, delta_y)

        # Deadzone logic to kill infinite creeping
        if distance > 0.15:
            current_smoothing = 0.6  # Fast snap
        elif distance < 0.02:
            current_smoothing = 0.0  # Deadzone freeze
        else:
            current_smoothing = 0.1  # Micro-correction

        smooth_x += delta_x * current_smoothing
        smooth_y += delta_y * current_smoothing

        # Apply calibration offset
        active_x = smooth_x - offset_x
        active_y = smooth_y - offset_y

        # Clamp the active zone
        active_x = max(-1.0, min(1.0, active_x))
        active_y = max(-1.0, min(1.0, active_y))

        # --- MAP TO 3D PHYSICAL TABLE (MM) ---
        target_x = active_x * MAX_REACH_X
        target_z = active_y * MAX_REACH_Z
        target_y = FIXED_Y

        # Diagnostic print for the Freeze Test
        print(f"RawEye: {raw_eye_x:5.2f} | Yaw: {head_yaw:5.1f} || X:{target_x:6.1f} | Z:{target_z:6.1f}")

        # ==========================================
        # VISUALS & COMMUNICATION
        # ==========================================
        for idx in [362, 263, 467, 374]:
            cv2.circle(frame, (int(landmarks[idx].x * w), int(landmarks[idx].y * h)), 2, (0, 0, 255), -1)
        cv2.circle(frame, (int(landmarks[473].x * w), int(landmarks[473].y * h)), 3, (0, 255, 0), -1)
        
        for idx in [133, 33, 159, 145]:
            cv2.circle(frame, (int(landmarks[idx].x * w), int(landmarks[idx].y * h)), 2, (0, 0, 255), -1)
        cv2.circle(frame, (int(landmarks[468].x * w), int(landmarks[468].y * h)), 3, (0, 255, 0), -1)

        if success:
            cv2.drawFrameAxes(frame, cam_matrix, dist_matrix, rvec, tvec, 150, 2)

        # SEND TO ESP32
        if ser:
            if active_x < -0.6 and active_y < -0.6:
                ser.write(b'U') 
            elif active_x > 0.6 and active_y < -0.6:
                ser.write(b'V')
            else:
                data_packet = f"{target_x:.1f},{target_y:.1f},{target_z:.1f}\n"
                ser.write(data_packet.encode('utf-8'))

    # --- KEYBOARD LOGIC ---
    if not is_headless:
        cv2.imshow('HackABull Eye Tracker', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            break
        elif key == ord(' '): 
            offset_x = smooth_x
            offset_y = smooth_y
            print(f">>> CALIBRATED! Center set to current gaze. <<<")

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()