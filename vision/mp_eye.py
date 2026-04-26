import cv2
import mediapipe as mp
import os
import serial
import time
import threading
import math

# ==========================================
# PHYSICAL MEASUREMENTS (RPi Cam Setup)
# ==========================================
CAM_HEIGHT = 20.0       # cm above the table surface
CAM_TO_ROBOT_Z = 10.0   # cm horizontal distance forward to robot base
CAM_TO_ROBOT_Y = 10.0   # cm vertical distance DOWN from camera to robot base
CAM_TILT_DEG = 0.0      # Perfectly horizontal

# Raspberry Pi Camera V2 FOV Specs
H_FOV = 62.2 
V_FOV = 48.8 
# ==========================================

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# --- Fixed Camera Class (THREAD SAFE) ---
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
        # Keep looping indefinitely until the thread is stopped
        while not self.stopped:
            ret, frame = self.stream.read()
            if ret:
                self.ret = ret
                self.frame = frame
            else:
                # If TCP lags, don't crash. Just wait a tiny bit for the next frame
                time.sleep(0.01) 

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        # Wait for the thread to safely finish its last loop before releasing the camera
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.stream.release()

# --- Serial Setup (Hardware UART AMA0) ---
ESP_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 115200
try:
    ser = serial.Serial(ESP_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Connected to ESP32 on {ESP_PORT}!")
except Exception as e:
    print(f"WARNING: Could not connect to ESP32 on {ESP_PORT}. Running vision only.")
    ser = None

# --- Mediapipe Setup ---
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True, 
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

print("Connecting to fast RPi camera TCP stream...")
cam = CameraStream().start()
time.sleep(1) 

if not cam.ret:
    print("Failed to open stream. Check your TCP connection.")
    cam.stop()
    exit()

print("Stream connected! Tracking gaze...")

# --- Calibration Offsets ---
offset_x = 0.0
offset_y = 0.0

while True:
    ret, frame = cam.read()
    
    # If the frame drops, don't crash the script, just skip this loop iteration
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
        
        # 1. Grab Landmarks (Pixel Space)
        ix, iy = landmarks[362].x * w, landmarks[362].y * h  # Inner corner
        ox, oy = landmarks[263].x * w, landmarks[263].y * h  # Outer corner
        rx, ry = landmarks[473].x * w, landmarks[473].y * h  # Iris
        bx, by = landmarks[336].x * w, landmarks[336].y * h  # Brow bone
        
        # 2. Find Head Tilt Angle & Un-tilt
        dx, dy = ox - ix, oy - iy
        angle = math.atan2(dy, dx)
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        cx, cy = (ix + ox) / 2.0, (iy + oy) / 2.0
        
        def rotate_point(px, py):
            return (
                cx + (px - cx) * cos_a - (py - cy) * sin_a,
                cy + (px - cx) * sin_a + (py - cy) * cos_a
            )
            
        rx_rot, ry_rot = rotate_point(rx, ry)
        bx_rot, by_rot = rotate_point(bx, by)
        
        # 3. Calculate Normalized Gaze (Distance-Invariant)
        eye_width_pix = max(1.0, math.hypot(dx, dy)) # Prevent div by zero 
        brow_dist_pix = max(1.0, abs(cy - by_rot))
        
        norm_x = (rx_rot - cx) / eye_width_pix
        norm_y = (ry_rot - by_rot) / (brow_dist_pix * 1.5)
        
        # 4. Apply Spacebar Calibration Offset
        final_x = norm_x - offset_x
        final_y = norm_y - offset_y
        final_x = max(-0.5, min(0.5, final_x))
        final_y = max(-0.5, min(0.5, final_y))

        # --- THE 3D SPATIAL MATH ---
        
        # Convert normalized gaze to radians based on RPi Cam FOV
        yaw = final_x * math.radians(H_FOV)
        pitch = (final_y * math.radians(V_FOV)) + math.radians(CAM_TILT_DEG)

        target_x = 0.0
        target_y = 0.0
        target_z = 0.0

        # Only calculate table intersection if looking *down* at the table
        if pitch > 0.01: 
            # Raycast to the table
            dist_on_table = CAM_HEIGHT / math.tan(pitch)
            
            # 3D Coordinates relative to Camera
            table_x_cam = dist_on_table * math.sin(yaw)
            table_z_cam = dist_on_table * math.cos(yaw)

            # Shift origin to the Robot Base
            target_x = table_x_cam
            target_z = table_z_cam - CAM_TO_ROBOT_Z
            
            # Y-Axis offset for floating robot base
            target_y = -(CAM_HEIGHT - CAM_TO_ROBOT_Y)
        
        # Make the print statement a little cleaner for your terminal
        print(f"X:{target_x:5.1f} | Y:{target_y:5.1f} | Z:{target_z:5.1f}")

        # --- SEND TO ESP32 (WITH TRIGGERS) ---
        if ser:
            if final_x < -0.3 and final_y < -0.3:
                ser.write(b'U') # Up-Left Trigger
            elif final_x > 0.3 and final_y < -0.3:
                ser.write(b'V') # Up-Right Trigger
            else:
                # Sending X, Y, and Z to the ESP32!
                data_packet = f"{target_x:.1f},{target_y:.1f},{target_z:.1f}\n"
                ser.write(data_packet.encode('utf-8'))

        # --- VISUAL FEEDBACK ---
        cv2.line(frame, (int(ix), int(iy)), (int(ox), int(oy)), (255, 0, 0), 1)
        cv2.circle(frame, (int(rx), int(ry)), 4, (0, 255, 0), -1)
        cv2.circle(frame, (int(bx), int(by)), 4, (0, 165, 255), -1)

    # --- KEYBOARD LOGIC ---
    if not is_headless:
        cv2.imshow('HackABull Tracker', frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'): 
            break
        elif key == ord(' '): 
            offset_x = norm_x
            offset_y = norm_y
            print(f">>> CALIBRATED! Center set. <<<")

# Safely shut down everything
cam.stop()
if ser: ser.close()
if not is_headless: cv2.destroyAllWindows()