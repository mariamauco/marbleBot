#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gpiozero import DigitalInputDevice, OutputDevice # controls the gpio pins
import time

# sensor's filter, which color channel (RGB) is measured
s2 = OutputDevice(23)
s3 = OutputDevice(24)
# pin  that reads sensor's output frequency
signal = DigitalInputDevice(25, pull_up=True)
# num of signal cycles to measure for calculating
# each color's frequency
CYCLES = 10

# uses s2 and s3 to measure each color channel
def get_color_reading():

    # get red value
    s2.off()
    s3.off()
    time.sleep(0.3)
    start = time.time()
    for impulse_count in range(CYCLES):
        signal.wait_for_inactive()
    duration = time.time() - start
    red = CYCLES / duration

    s2.off()
    s3.on()
    time.sleep(0.3)
    start = time.time()
    for impulse_count in range(CYCLES):
        signal.wait_for_inactive()
    duration = time.time() - start
    blue = CYCLES / duration

    s2.on()
    s3.on()
    time.sleep(0.3)
    start = time.time()
    for impulse_count in range(CYCLES):
        signal.wait_for_inactive()
    duration = time.time() - start
    green = CYCLES / duration

    return red, green, blue

def color_detector():
    # ros node
    rospy.init_node('color_detector', anonymous=True)
    color_pub = rospy.Publisher('/marble_color', String, queue_size=10)
    

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
        

def endprogram():
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        color_detector()
    except rospy.ROSInterruptException:
        pass
    finally:
        endprogram()