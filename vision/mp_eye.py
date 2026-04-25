import cv2
import mediapipe as mp
import os
import serial
import time
import threading

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- Fixed Camera Class (Zero Lag) ---
class CameraStream:
    def __init__(self, src='tcp://127.0.0.1:8888'):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self.ret, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.stream.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()
# ------------------------------------------

# --- Serial Setup ---
ESP_PORT = '/dev/ttyUSB0' 
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

while True:
    ret, frame = cam.read()
    if not ret: break

    h, w, _ = frame.shape
    small_frame = cv2.resize(frame, (320, 240))
    rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
    
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        iris = landmarks[473]
        inner_corner = landmarks[362]
        outer_corner = landmarks[263]
        top_edge = landmarks[386]
        bottom_edge = landmarks[374]
        
        # --- GAZE MATH (RAW RATIOS) ---
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

        # --- CARTESIAN MAPPING (-1.0 to 1.0) ---
        # Map X: 0.0 (left) to 1.0 (right) -> -1.0 to 1.0
        cart_x = (ratio_x * 2.0) - 1.0
        
        # Map Y: 0.0 (top) to 1.0 (bottom) -> 1.0 to -1.0 (Inverted so bottom is -1)
        cart_y = -((ratio_y * 2.0) - 1.0)

        # Clamp values to strictly stay within the -1.0 and 1.0 bounds
        cart_x = max(-1.0, min(1.0, cart_x))
        cart_y = max(-1.0, min(1.0, cart_y))

        # --- DETERMINE DIRECTION (Using the new scale) ---
        # Thresholds: -0.2 to 0.2 is the "deadzone" for looking straight ahead
        if cart_x < -0.20: x_dir = "left"
        elif cart_x > 0.20: x_dir = "right"
        else: x_dir = "center"

        if cart_y < -0.20: y_dir = "down"
        elif cart_y > 0.20: y_dir = "up"
        else: y_dir = "center"

        print(f"Looking {y_dir} and {x_dir} | X:{cart_x:.2f} Y:{cart_y:.2f}")

        # --- DRAWING THE DOTS ---
        for lm in [inner_corner, outer_corner, top_edge, bottom_edge]:
            cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, (0, 0, 255), -1)
        cv2.circle(frame, (int(iris.x * w), int(iris.y * h)), 4, (0, 255, 0), -1)

        # --- SEND TO ESP32 ---
        if ser:
            # Sending the perfectly mapped Cartesian coordinates
            data_packet = f"{cart_x:.2f},{cart_y:.2f}\n"
            ser.write(data_packet.encode('utf-8'))

    if not is_headless:
        cv2.imshow('Eye Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()