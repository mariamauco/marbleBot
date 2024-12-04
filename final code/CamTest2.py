import cv2

for i in range(19, 38):  # Adjust range based on your devices
    cap = cv2.VideoCapture(f"/dev/video{i}")
    if cap.isOpened():
        print(f"Device /dev/video{i} is accessible.")
        ret, frame = cap.read()
        if ret:
            print(f"Device /dev/video{i} is capturing frames.")
            cap.release()
            break
        else:
            print(f"Device /dev/video{i} failed to capture frames.")
            cap.release()
    else:
        print(f"Device /dev/video{i} is not accessible.")
