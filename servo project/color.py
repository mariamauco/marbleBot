#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import gpiod
import time

chip = gpiod.chip("gpiochip0")
# sensor's filter, which color channel (RGB) is measured
s2 = chip.get_line(5)
s3 = chip.get_line(6)
# pin  that reads sensor's output frequency
signal = chip.get_line(13)
# num of signal cycles to measure for calculating
# each color's frequency
CYCLES = 10

COLOR_MARKERS = {
    "white": 1,
    "yellow": 2,
    "red": 3,
    "green": 4,
    "blue": 5,
    "black": 6,
    "UNIDENTIFIED": 0 
}

def setup():
    config = gpoid.line_request()
    config.consumer = "s2_s3_output"
    config.request_type = gpoid.line_request.DIRECTION_OUTPUT
    s2.request(config)
    s3.request(config)

    config.consumer = "signal_input"
    config.request_type = gpoid.line_request.DIRECTION_INPUT
    signal.request(config)

# uses s2 and s3 to measure each color channel
def get_color_reading():

    # get red value
    s2.set_value(0)
    s3.set_value(0)
    time.sleep(0.3)
    

    start = time.time()
    for i in range(CYCLES):
        while signal.get_value() == 1:
            pass
        while signal.get_value() == 0:
            pass
    duration = time.time() - start
    red = CYCLES / duration

    # set to blue channel
    s2.set_value(0)
    s3.set_value(1)
    time.sleep(0.3)
    start = time.time()
    for i in range(CYCLES):
        while signal.get_value() == 1:
            pass
        while signal.get_value() == 0:
            pass
    duration = time.time() - start
    blue = CYCLES / duration

    # green value
    s2.set_value(1)
    s3.set_value(1)
    time.sleep(0.3)
    start = time.time()
    for i in range(CYCLES):
        while signal.get_value() == 1:
            pass
        while signal.get_value() == 0:
            pass
    duration = time.time() - start
    green = CYCLES / duration

    return red, green, blue

def detect_color(r, g, b):
    high = 200
    low = 50

    if r > high and g > high and b > high:
        return "white"
    elif r > high and g > high and b < low:
        return "yellow"
    elif r > g and r > b and r > high:
        return "red"
    elif g > r and g > b and g > high:
        return "green"
    elif b > r and b > g and b > high:
        return "blue"
    elif r < low and r < low and r < low:
        return "black"
    else:
        return "UNIDENTIFIED"

def color_detector():
    # ros node
    rospy.init_node('color_detector', anonymous=True)
    color_pub = rospy.Publisher('/marble_color', String, queue_size=10)
    marker_pub = rospy.Publisher('/marker_id', String, queue_size=10)
    
    setup()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        red, green, blue = get_color_reading()
        color = detect_color(red, green, blue)
        marker_id = COLOR_MARKERS.get(color, 0)
             
        
        color_pub.publish(color)
        marker_pub.publish(marker_id)
        print(f"{color} detected, looking for marker {marker_id}.")

        rate.sleep()

if __name__ == '__main__':
    try:
        color_detector()
    except rospy.ROSInterruptException:
        pass
    finally:
        s2.release()
        s3.release()
        signal.release()