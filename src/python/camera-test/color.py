import cv2
import numpy as np

def track_colored_object(frame, lower_bound, upper_bound):
    """Tracks a colored object in a given frame.

    Args:
        frame: The input frame from the camera.
        lower_bound: The lower bound of the color range in HSV format.
        upper_bound: The upper bound of the color range in HSV format.

    Returns:
        x, y, radius: The x and y coordinates of the object's center and its radius.
    """

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the specified color range
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Apply morphological operations to remove noise and improve detection
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables for object center and radius
    x, y, radius = 0, 0, 0

    # If at least one contour is found
    if len(contours) > 0:
        # Find the largest contour
        c = max(contours, key=cv2.contourArea)

        # Compute the minimum enclosing circle of the contour
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        # Draw a circle around the object
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

    return x, y, radius

# Define color range for tracking (example: blue)
lower_blue = np.array([100, 100, 100])
upper_blue = np.array([130, 255, 255])

# Open the camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    # Track the colored object
    x, y, radius = track_colored_object(frame, lower_blue, upper_blue)

    # Display the frame
    cv2.imshow("Object Tracking", frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()