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

# Gesture Sensitivity
EYE_CLOSED_THRESHOLD = 0.010  # Below this = eyes closed
BROW_RAISE_THRESHOLD = 0.120  # Above this = eyebrows raised
EYE_CLOSE_DURATION = 2.0      # Seconds to trigger claw close

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- SERIAL SETUP ---
try:
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
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

# Gesture State
claw_open = True  # Starts closed (G0)
eye_close_start_time = None

# Calibration Deltas
calib_total_yaw, calib_total_pitch = 0.0, 0.0

print("\n" + "="*50)
print("ACTION: STARE AT THE ROBOT BASE AND HIT SPACE")
print("GESTURES: RAISE BROWS (OPEN) | CLOSE EYES 2S (CLOSE)")
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
        
        # --- VISUAL DIAGNOSTICS ---
        pose_landmarks = [1, 152, 33, 263, 61, 291]
        gaze_landmarks = [468, 473, 159, 386, 247, 467, 133, 362]
        gesture_landmarks = [105, 334, 145, 374] # Brows and bottom lids

        for idx in pose_landmarks + gaze_landmarks + gesture_landmarks:
            point = mesh[idx]
            cx, cy = int(point.x * w), int(point.y * h)
            color = (255, 255, 0) if idx in gaze_landmarks else (255, 0, 0)
            cv2.circle(frame, (cx, cy), 2, color, -1)

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

        # 2. GAZE LOGIC
        def get_gaze_delta(iris_idx, x_ref_idx, y_ref_idx, l_corner, r_corner):
            scale = abs(mesh[r_corner].x - mesh[l_corner].x)
            dx = (mesh[iris_idx].x - mesh[x_ref_idx].x) / (scale + 1e-6)
            dy = (mesh[iris_idx].y - mesh[y_ref_idx].y) / (scale + 1e-6)
            return dx, dy

        rx, ry = get_gaze_delta(468, 159, 247, 133, 33)
        lx, ly = get_gaze_delta(473, 386, 467, 362, 263)
        
        total_yaw = math.radians(hy) + ((lx + rx) / 2.0) * EYE_SENS_X
        total_pitch = math.radians(hp) - ((ly + ry) / 2.0) * EYE_SENS_Y

        # 3. GESTURE LOGIC (Stateful)
        # EAR (Eye Aspect Ratio) proxy using vertical lid distance
        left_ear = abs(mesh[159].y - mesh[145].y)
        right_ear = abs(mesh[386].y - mesh[374].y)
        avg_ear = (left_ear + right_ear) / 2.0
        
        # Brow Raise distance (brow to upper lid)
        left_brow_dist = abs(mesh[105].y - mesh[159].y)
        right_brow_dist = abs(mesh[334].y - mesh[386].y)
        avg_brow = (left_brow_dist + right_brow_dist) / 2.0

        # Claw Open Logic: Eyebrow Raise (Set to True, else no change)
        if avg_brow < BROW_RAISE_THRESHOLD and avg_ear > EYE_CLOSED_THRESHOLD:
            print(">>> NEUTRAL <<<")
        else:
            if avg_brow > BROW_RAISE_THRESHOLD:
                claw_open = True
            # Claw Close Logic: Long Eye Closure (Set to False after 2s, else no change)
            if avg_ear < EYE_CLOSED_THRESHOLD:
                if eye_close_start_time is None:
                    eye_close_start_time = time.time()
                elif time.time() - eye_close_start_time >= EYE_CLOSE_DURATION:
                    claw_open = False
            else:
                eye_close_start_time = None

        

        # 4. MATH & SMOOTHING
        delta_yaw = total_yaw - calib_total_yaw
        delta_pitch = total_pitch - calib_total_pitch
        raw_x = CAM_HEIGHT_ABOVE_TABLE * math.tan(delta_yaw)
        raw_z = CAM_HEIGHT_ABOVE_TABLE * math.tan(delta_pitch)

        smooth_x = (ALPHA * raw_x) + (1 - ALPHA) * smooth_x
        smooth_z = (ALPHA * raw_z) + (1 - ALPHA) * smooth_z

        # --- TERMINAL OUTPUT ---
        status = "OPEN" if claw_open else "CLOSED"
        print(f"X: {smooth_x:6.1f} | Z: {smooth_z:6.1f} | G: {status} | EAR: {avg_ear:.3f} | ARVBROW:{avg_brow:.3f}", end='\r')

        # 5. SEND TO ESP32 (Format: X{.2f}Z:{.2f}G{})
        if ser:
            data = f"X{smooth_x:.2f}Z:{smooth_z:.2f}G{int(claw_open)}\n"
            ser.write(data.encode())
            print(f"Sent: {data.strip()}")

    if not is_headless:
        cv2.imshow('HackABull Control', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            calib_total_yaw = total_yaw
            calib_total_pitch = total_pitch
            print("\n>>> ZEROED <<<\n")
        elif key == ord('q'): break

cam.stop()
if ser: ser.close()
cv2.destroyAllWindows()