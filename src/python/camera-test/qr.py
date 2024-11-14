import cv2
import numpy as np
from pyzbar.pyzbar import decode

# Known physical size of QR code (in cm)
KNOWN_WIDTH = 10
FOCAL_LENGTH = 500

def find_distance(decoded_objects, image):
    for obj in decoded_objects:
        points = obj.polygon
        if len(points) > 4:
            points = np.array(points)
            points = points.reshape((-1, 1, 2))
            cv2.polylines(image, [points], True, (0, 255, 0), 3)

            # Calculate QR code width in pixels
            pixel_width = np.linalg.norm(points[0] - points[1])

            # Calculate distance using the known physical width
            distance = (KNOWN_WIDTH * FOCAL_LENGTH) / pixel_width
            return distance

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        if ret:
            decoded_objects = decode(frame)
            
            if len(decoded_objects) > 0:
                distance = find_distance(decoded_objects, frame)

                if distance is not None:
                    cv2.putText(frame, f"Distance: {distance:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                cv2.imshow("QR Code Distance", frame)
            else:
                cv2.imshow("QR Code Distance", frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()