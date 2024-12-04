#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gpiod
import time

# GPIO configuration
chip = gpiod.Chip('gpiochip4')
s2 = chip.get_line(13)
s3 = chip.get_line(5)
signal = chip.get_line(6)
CYCLES = 10

# Color marker mappings
COLOR_MARKERS = {
    "white": 1,
    "yellow": 2,
    "red": 3,
    "green": 4,
    "blue": 5,
    "black": 6,
    "UNIDENTIFIED": 0 
}

# GPIO setup
def setup_gpio():
    s2.request(consumer="s2_s3_output", type=gpiod.LINE_REQ_DIR_OUT)
    s3.request(consumer="s2_s3_output", type=gpiod.LINE_REQ_DIR_OUT)
    signal.request(consumer="signal_input", type=gpiod.LINE_REQ_DIR_IN)

# Color detection logic
def get_color_reading():
    # Get red value
    s2.set_value(0)
    s3.set_value(0)
    time.sleep(0.3)
    red = measure_frequency()

    # Get blue value
    s2.set_value(0)
    s3.set_value(1)
    time.sleep(0.3)
    blue = measure_frequency()

    # Get green value
    s2.set_value(1)
    s3.set_value(1)
    time.sleep(0.3)
    green = measure_frequency()

    print(f"Red: {red}, Green: {green}, Blue: {blue}")
    return red, green, blue

def measure_frequency():
    start = time.time()
    for _ in range(CYCLES):
        while signal.get_value() == 0:
            pass
        while signal.get_value() == 1:
            pass
    duration = time.time() - start
    return CYCLES / duration

def detect_color(r, g, b):
    high = 1500
    low = 700
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
    elif r < low and g < low and b < low:
        return "black"
    else:
        return "UNIDENTIFIED"

# ROS2 Node for Color Detection
class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.color_pub = self.create_publisher(String, '/marble_color', 10)
        self.marker_pub = self.create_publisher(String, '/marker_id', 10)
        setup_gpio()
        self.timer = self.create_timer(1.0, self.color_detection_callback)

    def color_detection_callback(self):
        red, green, blue = get_color_reading()
        color = detect_color(red, green, blue)
        marker_id = COLOR_MARKERS.get(color, 0)

        # Publish detected color
        color_msg = String()
        color_msg.data = color
        self.color_pub.publish(color_msg)

        # Publish marker ID
        marker_msg = String()
        marker_msg.data = str(marker_id)
        self.marker_pub.publish(marker_msg)

        self.get_logger().info(f"Detected color: {color}, Marker ID: {marker_id}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Color Detector node.')
    finally:
        s2.release()
        s3.release()
        signal.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
