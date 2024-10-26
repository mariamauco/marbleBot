# spin robot to scan the qr codes

# takes in the /marble_color from color_detect,
# and scans each code until it finds matching color

# needs to work with movement node

import visp3 as vp
import cv2
import rospy

cam = cv2.VideoCapture(0)

while True:
    rospy.init_node('qr_scan', anonymous=True)
    qr_pub = rospy.Publisher('/qr_data', String, queue_size=10)

    frame = cam.read()

    detector = cv2.QRCodeDetector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    data, points, unused = detector(gray)

    if data:
        qr_pub.publish(data)
        print("This bin is for ", data, " marbles.")

cam.release()