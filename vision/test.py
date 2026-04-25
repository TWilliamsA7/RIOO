import cv2

# Initialize the camera
# On a Pi 3, the camera is usually assigned to index 0
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open the PiCam. Check your ribbon cable!")
    exit()

print("Camera is active! Press 'q' to close the window.")

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame.")
        break
        
    cv2.imshow('PiCam Test', frame)
    
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()