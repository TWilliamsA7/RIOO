import cv2

# Open the camera
cap = cv2.VideoCapture(0)

# 1. Force a lower, stable resolution BEFORE reading
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 2. Force MJPEG compression so the Pi 3 doesn't choke on raw YUV data
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

print("Camera is active! Attempting to capture a frame...")

ret, frame = cap.read()

if not ret:
    print("ERROR: Frame dropped.")
else:
    print(f"SUCCESS! Grabbed a frame of size: {frame.shape}")
    # If in mp_eye.py, your MediaPipe logic would continue here...

cap.release()