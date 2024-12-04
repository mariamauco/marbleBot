import cv2

cap = cv2.VideoCapture(0)  # Change '0' to the appropriate device index if needed
if not cap.isOpened():
    print("Failed to open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
