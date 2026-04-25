import cv2
import mediapipe as mp
import os
import serial
import time

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

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

print("Connecting to camera stream...")
cap = cv2.VideoCapture('tcp://127.0.0.1:8888')

if not cap.isOpened():
    print("Failed to open stream.")
    exit()

print("Stream connected! Tracking eyes...")

# --- OPTIMIZATION VARIABLES ---
frame_count = 0
PROCESS_EVERY_N_FRAMES = 3  # Tweak this. 2 = more CPU, 4 = less CPU.
last_x, last_y = 0.0, 0.0
# ------------------------------

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break

    frame_count += 1

    # Only run the heavy math every Nth frame
    if frame_count % PROCESS_EVERY_N_FRAMES == 0:
        
        # 1. DOWNSCALE: Shrink the image to 320x240 for faster AI processing
        small_frame = cv2.resize(frame, (320, 240))
        
        # 2. CONVERT: Change to RGB
        rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        # 3. PROCESS
        results = face_mesh.process(rgb_frame)

        if results.multi_face_landmarks:
            eye_landmark = results.multi_face_landmarks[0].landmark[473]
            
            last_x = (eye_landmark.x - 0.5) * 2
            last_y = (eye_landmark.y - 0.5) * 2
            
            print(f"X: {last_x:.2f}, Y: {last_y:.2f}")

            # 4. SEND DATA: Only send to ESP32 when we have fresh coordinates
            if ser:
                data_packet = f"{last_x:.2f},{last_y:.2f}\n"
                ser.write(data_packet.encode('utf-8'))

    # Display the FULL SIZE frame if a monitor is plugged in
    if not is_headless:
        cv2.imshow('Eye Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()