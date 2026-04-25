import cv2

# Initialize the camera using the V4L2 backend
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Error: Could not open the PiCam.")
    exit()

print("Camera is active! Attempting to capture a frame...")

# Burn through the first 30 frames to let exposure auto-adjust
for i in range(30):
    ret, frame = cap.read()

if ret:
    # Save the frame to a file
    cv2.imwrite('test_snapshot.jpg', frame)
    print("Success! Snapshot saved as 'test_snapshot.jpg'. Check your folder.")
else:
    print("Failed to capture frame.")

# Clean up without GUI functions
cap.release()