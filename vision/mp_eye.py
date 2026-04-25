import cv2
import mediapipe as mp
import os

is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True, 
    min_detection_confidence=0.4,
    min_tracking_confidence=0.4
)

print("Connecting to camera stream...")
# Connect to the TCP stream we started in the other terminal
cap = cv2.VideoCapture('tcp://127.0.0.1:8888')

if not cap.isOpened():
    print("Failed to open stream.")
    exit()

print("Stream connected! Tracking eyes...")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: 
        print("Dropped frame.")
        break

    # Convert to RGB for MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        # Get the left eye pupil
        eye_landmark = results.multi_face_landmarks[0].landmark[473]
        
        # Normalize (0 to 1 -> -1 to 1)
        x_norm = (eye_landmark.x - 0.5) * 2
        y_norm = (eye_landmark.y - 0.5) * 2
        
        print(f"X: {x_norm:.2f}, Y: {y_norm:.2f}")

    if not is_headless:
        cv2.imshow('Eye Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if not is_headless:
    cv2.destroyAllWindows()