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
        # FORCE OpenCV to drop old frames instantly
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self.ret, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            # No sleep command. Consume the TCP buffer as fast as possible.
            self.ret, self.frame = self.stream.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()
# ------------------------------------------

# --- Serial Setup ---
ESP_PORT = '/dev/ttyUSB0' # Change to /dev/ttyACM0 if needed
BAUD_RATE = 115200
try:
    ser = serial.Serial(ESP_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Connected to ESP32 on {ESP_PORT}!")
except Exception as e:
    print(f"WARNING: Could not connect to ESP32 on {ESP_PORT}. Running vision only.")
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
time.sleep(1) # Let the camera warm up

if not cam.ret:
    print("Failed to open stream.")
    cam.stop()
    exit()

print("Stream connected! Tracking gaze...")

while True:
    ret, frame = cam.read()
    if not ret: break

    h, w, _ = frame.shape
    
    # Downscale for much faster AI processing on the Pi
    small_frame = cv2.resize(frame, (320, 240))
    rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
    
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        # Left eye (from the image's perspective)
        iris = landmarks[473]
        inner_corner = landmarks[362]
        outer_corner = landmarks[263]
        
        # --- THE EQUATOR MATH ---
        # 1. The X-axis (Width)
        eye_width = outer_corner.x - inner_corner.x
        if eye_width != 0:
            ratio_x = (iris.x - inner_corner.x) / eye_width
        else:
            ratio_x = 0.5
            
        # 2. The Y-axis (Equator) 
        equator_y = (inner_corner.y + outer_corner.y) / 2.0
        
        if eye_width != 0:
            raw_y = (iris.y - equator_y) / eye_width
        else:
            raw_y = 0.0

        # --- CARTESIAN MAPPING (-1.0 to 1.0) ---
        cart_x = (ratio_x * 2.0) - 1.0
        
        # Multiplier to scale the tiny vertical iris movements
        Y_SENSITIVITY = 8.0 
        
        # Invert the Y so looking down becomes negative
        cart_y = -(raw_y * Y_SENSITIVITY)

        # Clamp values to strictly stay within bounds
        cart_x = max(-1.0, min(1.0, cart_x))
        cart_y = max(-1.0, min(1.0, cart_y))

        # --- DETERMINE DIRECTION (Strict Binary/Center) ---
        check_x = round(cart_x, 2)
        if check_x < 0.0: x_dir = "left"
        elif check_x > 0.0: x_dir = "right"
        else: x_dir = "center"

        check_y = round(cart_y, 2)
        if check_y < 0.0: y_dir = "down"
        elif check_y > 0.0: y_dir = "up"
        else: y_dir = "center"

        print(f"Looking {y_dir} and {x_dir} | X:{cart_x:.2f} Y:{cart_y:.2f}")

        # --- DRAWING THE DOTS & EQUATOR ---
        cv2.circle(frame, (int(inner_corner.x * w), int(inner_corner.y * h)), 3, (0, 0, 255), -1)
        cv2.circle(frame, (int(outer_corner.x * w), int(outer_corner.y * h)), 3, (0, 0, 255), -1)
        
        # Blue Equator line
        cv2.line(frame, 
                 (int(inner_corner.x * w), int(inner_corner.y * h)), 
                 (int(outer_corner.x * w), int(outer_corner.y * h)), 
                 (255, 0, 0), 1)

        # Green Iris dot
        cv2.circle(frame, (int(iris.x * w), int(iris.y * h)), 4, (0, 255, 0), -1)

        # --- SEND TO ESP32 ---
        if ser:
            data_packet = f"{cart_x:.2f},{cart_y:.2f}\n"
            ser.write(data_packet.encode('utf-8'))

    if not is_headless:
        cv2.imshow('Eye Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()