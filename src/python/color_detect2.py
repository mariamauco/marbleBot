#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import gpiod
import time

chip = gpiod.chip("gpiochip0")
# sensor's filter, which color channel (RGB) is measured
s2 = chip.get_line(23)
s3 = chip.get_line(24)
# pin  that reads sensor's output frequency
signal = chip.get_line(25)
# num of signal cycles to measure for calculating
# each color's frequency
CYCLES = 10

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

def color_detector():
    # ros node
    rospy.init_node('color_detector', anonymous=True)
    color_pub = rospy.Publisher('/marble_color', String, queue_size=10)
    
    setup()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        red, green, blue = get_color_reading()

        high = 200
        low = 50

        if red > high and green > high and blue > high:
            color = "white"
        elif red > high and green > high and blue < low:
            color = "yellow"
        elif red > green and red > blue and red > high:
            color = "red"
        elif green > red and green > blue and green > high:
            color = "green"
        elif blue > red and blue > green and blue > high:
            color = "blue"
        elif red < low and red < low and red < low:
            color = "black"
        else:
            color = "idk"
        
        color_pub.publish(color)
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