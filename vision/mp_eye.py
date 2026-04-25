import cv2
import mediapipe as mp
import os
import serial
import time
import threading

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- NEW: Multithreaded Camera Class ---
class CameraStream:
    def __init__(self, src='tcp://127.0.0.1:8888'):
        self.stream = cv2.VideoCapture(src)
        self.ret, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        # Start the thread to read frames in the background
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        # Keep looping infinitely until the thread is stopped
        while not self.stopped:
            self.ret, self.frame = self.stream.read()
            # A tiny sleep prevents this thread from eating 100% of the CPU
            time.sleep(0.01) 

    def read(self):
        # Return the absolute newest frame
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()
# ---------------------------------------

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
# Start our threaded camera!
cam = CameraStream().start()
time.sleep(1) # Let the camera warm up

if not cam.ret:
    print("Failed to open stream.")
    cam.stop()
    exit()

print("Stream connected! Tracking eyes with zero lag...")

while True:
    # Grab the absolute freshest frame from the background thread
    ret, frame = cam.read()
    if not ret: break

    h, w, _ = frame.shape
    small_frame = cv2.resize(frame, (320, 240))
    rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
    
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        eye_landmark = results.multi_face_landmarks[0].landmark[473]
        
        x_norm = (eye_landmark.x - 0.5) * 2
        y_norm = (eye_landmark.y - 0.5) * 2
        
        pixel_x = int(eye_landmark.x * w)
        pixel_y = int(eye_landmark.y * h)
        
        cv2.circle(frame, (pixel_x, pixel_y), 6, (0, 255, 0), -1)

        print(f"X: {x_norm:.2f}, Y: {y_norm:.2f}")
        if ser:
            data_packet = f"{x_norm:.2f},{y_norm:.2f}\n"
            ser.write(data_packet.encode('utf-8'))

    if not is_headless:
        cv2.imshow('Eye Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()