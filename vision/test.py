import cv2

print("Connecting to local camera stream...")

# Connect to the TCP stream hosted by rpicam-vid
cap = cv2.VideoCapture('tcp://127.0.0.1:8888')

if not cap.isOpened():
    print("ERROR: Could not connect to the stream.")
else:
    print("Stream connected! Attempting to capture a frame...")
    ret, frame = cap.read()
    
    if not ret:
        print("ERROR: Stream connected, but returned an empty frame.")
    else:
        print(f"SUCCESS! Grabbed a frame of size: {frame.shape}")

cap.release()