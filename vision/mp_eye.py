import cv2
import mediapipe as mp
import os
import serial
import time
import threading

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# ==========================================
# PHYSICAL REACH LIMITS
# ==========================================
# How far (in cm) can the arm reach from its center point?
MAX_REACH_X = 15.0  # 15cm left or right
MAX_REACH_Z = 15.0  # 15cm forward or backward
FIXED_Y = -10.0     # The table is 10cm below the robot base
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

print("Connecting to fast camera stream...")
cam = CameraStream().start()
time.sleep(1) 

if not cam.ret:
    print("Failed to open stream.")
    cam.stop()
    exit()

print("Stream connected! Tracking gaze...")

# Calibration Offsets
offset_x = 0.0
offset_y = 0.0

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
        
        # --- YOUR ORIGINAL EXACT LANDMARKS ---
        iris = landmarks[473]
        inner_corner = landmarks[362]
        outer_corner = landmarks[263]
        top_edge = landmarks[467]
        bottom_edge = landmarks[374]
        
        # --- YOUR ORIGINAL CARTESIAN MATH ---
        eye_width = outer_corner.x - inner_corner.x
        if eye_width != 0:
            ratio_x = (iris.x - inner_corner.x) / eye_width
        else:
            ratio_x = 0.5
            
        eye_height = bottom_edge.y - top_edge.y
        if eye_height != 0:
            ratio_y = (iris.y - top_edge.y) / eye_height
        else:
            ratio_y = 0.5

        cart_x = (ratio_x * 2.0) - 1.0
        cart_y = -((ratio_y * 2.0) - 1.0)

        cart_x = max(-1.0, min(1.0, cart_x))
        cart_y = max(-1.0, min(1.0, cart_y))

        # --- APPLY CALIBRATION OFFSET ---
        active_x = cart_x - offset_x
        active_y = cart_y - offset_y

        # Clamp the active zone so looking away doesn't break the arm
        active_x = max(-1.0, min(1.0, active_x))
        active_y = max(-1.0, min(1.0, active_y))

        # --- MAP TO 3D PHYSICAL TABLE (CM) ---
        target_x = active_x * MAX_REACH_X
        target_z = active_y * MAX_REACH_Z
        target_y = FIXED_Y

        print(f"Target -> X:{target_x:5.1f} | Y:{target_y:5.1f} | Z:{target_z:5.1f}")

        # --- YOUR ORIGINAL VISUAL DOTS ---
        for lm in [inner_corner, outer_corner, top_edge, bottom_edge]:
            cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, (0, 0, 255), -1)
        cv2.circle(frame, (int(iris.x * w), int(iris.y * h)), 4, (0, 255, 0), -1)

        # --- SEND TO ESP32 ---
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
        cv2.imshow('Eye Tracker', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            break
        elif key == ord(' '): 
            offset_x = cart_x
            offset_y = cart_y
            print(f">>> CALIBRATED! Center set to current gaze. <<<")

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()