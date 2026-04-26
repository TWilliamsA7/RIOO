import cv2
import mediapipe as mp
import os
import serial
import time
import threading
import numpy as np

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# ==========================================
# PHYSICAL REACH LIMITS (IN MILLIMETERS)
# ==========================================
MAX_REACH_X = 150.0  # 150mm left or right (15cm)
MAX_REACH_Z = 150.0  # 150mm forward or backward (15cm)
FIXED_Y = -100.0     # The table is 100mm below the robot base (-10cm)

# --- HEAD TRACKING WEIGHTS ---
# These control how much your head turn affects the final room coordinate. 
# If turning your head moves the arm the WRONG way, change the + to a - in the math below!
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

print("Stream connected! Tracking gaze and head pose...")

# Calibration Offsets & Smoothing
offset_x = 0.0
offset_y = 0.0
smooth_x = 0.0
smooth_y = 0.0
SMOOTHING_FACTOR = 0.15 

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
        # Extract the 6 key points for the 3D map
        face_2d_image = np.array([
            (landmarks[1].x * w, landmarks[1].y * h),     # Nose tip
            (landmarks[152].x * w, landmarks[152].y * h), # Chin
            (landmarks[33].x * w, landmarks[33].y * h),   # Left eye left corner
            (landmarks[263].x * w, landmarks[263].y * h), # Right eye right corner
            (landmarks[61].x * w, landmarks[61].y * h),   # Left Mouth corner
            (landmarks[291].x * w, landmarks[291].y * h)  # Right mouth corner
        ], dtype=np.float64)

        # Camera internals (Assuming standard webcam focal length)
        focal_length = 1.0 * w
        cam_matrix = np.array([[focal_length, 0, w / 2],
                               [0, focal_length, h / 2],
                               [0, 0, 1]], dtype=np.float64)
        dist_matrix = np.zeros((4, 1), dtype=np.float64)

        # Calculate Head Rotation Matrix
        success, rvec, tvec = cv2.solvePnP(face_3d_model, face_2d_image, cam_matrix, dist_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
        rmat, _ = cv2.Rodrigues(rvec)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)

        head_pitch = angles[0] # Nodding up/down (Degrees)
        head_yaw = angles[1]   # Turning left/right (Degrees)
        head_roll = angles[2]  # Tilting side to side (Degrees)

        # ==========================================
        # 2. EYE TRACKING (YOUR EXACT ORIGINAL MATH)
        # ==========================================
        iris = landmarks[473]
        inner_corner = landmarks[362]
        outer_corner = landmarks[263]
        top_edge = landmarks[467]
        bottom_edge = landmarks[374]
        
        eye_width = outer_corner.x - inner_corner.x
        ratio_x = (iris.x - inner_corner.x) / eye_width if eye_width != 0 else 0.5
            
        eye_height = bottom_edge.y - top_edge.y
        ratio_y = (iris.y - top_edge.y) / eye_height if eye_height != 0 else 0.5

        raw_eye_x = (ratio_x * 2.0) - 1.0
        raw_eye_y = -((ratio_y * 2.0) - 1.0)

        # ==========================================
        # 3. MERGING HEAD POSE AND GAZE
        # ==========================================
        # Combine the eye's movement with the head's physical rotation.
        # (If moving your head moves the arm the wrong way, change the + to a - )
        combined_x = raw_eye_x + (head_yaw * HEAD_YAW_WEIGHT)
        combined_y = raw_eye_y + (head_pitch * HEAD_PITCH_WEIGHT)

        # ==========================================
        # 4. SHOCK ABSORBER (SMOOTHING) & CALIBRATION
        # ==========================================
        smooth_x = (SMOOTHING_FACTOR * combined_x) + ((1.0 - SMOOTHING_FACTOR) * smooth_x)
        smooth_y = (SMOOTHING_FACTOR * combined_y) + ((1.0 - SMOOTHING_FACTOR) * smooth_y)

        active_x = smooth_x - offset_x
        active_y = smooth_y - offset_y

        # Clamp the active zone 
        active_x = max(-1.0, min(1.0, active_x))
        active_y = max(-1.0, min(1.0, active_y))

        # --- MAP TO 3D PHYSICAL TABLE (MM) ---
        target_x = active_x * MAX_REACH_X
        target_z = active_y * MAX_REACH_Z
        target_y = FIXED_Y

        print(f"Target -> X:{target_x:6.1f} | Y:{target_y:6.1f} | Z:{target_z:6.1f} (mm) | HeadYaw: {head_yaw:.1f}")

        # ==========================================
        # VISUALS & COMMUNICATION
        # ==========================================
        # Draw the Eye Dots
        for lm in [inner_corner, outer_corner, top_edge, bottom_edge]:
            cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, (0, 0, 255), -1)
        cv2.circle(frame, (int(iris.x * w), int(iris.y * h)), 4, (0, 255, 0), -1)

        # Draw the 3D Head Pose Axis (JUDGES WILL LOVE THIS)
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
            # Calibrate using the current smoothed combined position
            offset_x = smooth_x
            offset_y = smooth_y
            print(f">>> CALIBRATED! Center set to current gaze. <<<")

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()