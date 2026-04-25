import cv2
import mediapipe as mp
import os

# Set this to '1' if you are on the headless Pi
is_headless = os.environ.get('ROBOT_HEADLESS', '0') == '1'

# Initialize MediaPipe
mp_face_mesh = mp.solutions.face_mesh
# refine_landmarks=True is crucial for getting pupil accuracy
face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True, static_image_mode=False)

# Start Camera
cap = cv2.VideoCapture(1)

print("Starting eye tracking...")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 1. Convert BGR (OpenCV) to RGB (MediaPipe) - CRITICAL STEP
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # 2. Process
    results = face_mesh.process(rgb_frame)

    # 3. Tracking Logic
    if results.multi_face_landmarks:
        # Landmark 473 is the center of the left eye pupil
        # We access the first face detected [0]
        eye_landmark = results.multi_face_landmarks[0].landmark[473]
        
        # Normalize (0 to 1 -> -1 to 1)
        x_norm = (eye_landmark.x - 0.5) * 2
        y_norm = (eye_landmark.y - 0.5) * 2
        
        print(f"X: {x_norm:.2f}, Y: {y_norm:.2f}")

    # 4. Display (Only if not headless)
    if not is_headless:
        cv2.imshow('Eye Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
if not is_headless:
    cv2.destroyAllWindows()