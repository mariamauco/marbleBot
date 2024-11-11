import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gpiod
import re

# Constants for thresholds and target marker ID
TARGET_MARKER_ID = '123'  # Set your target marker ID here
DISTANCE_THRESHOLD = 0.2  # Distance threshold in meters for stopping near the marker
ORIENTATION_THRESHOLD = 5  # Rotation error threshold in degrees

# Define GPIO chip and lines for motor control
#CHIP_NAME = "/dev/gpiochip0"  # Default GPIO chip for Raspberry Pi
LEFT_FRONT = (23, 24)  # GPIO pins for left front motor (IN1, IN2)
LEFT_BACK = (12, 25)   # GPIO pins for left back motor (IN1, IN2)
RIGHT_FRONT = (16, 27) # GPIO pins for right front motor (IN1, IN2)
RIGHT_BACK = (17, 22)   # GPIO pins for right back motor (IN1, IN2)

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Initialize GPIO chip and lines
        self.chip = gpiod.Chip('gpiochip4')
        self.left_motor_pins = [LEFT_FRONT, LEFT_BACK]
        self.right_motor_pins = [RIGHT_FRONT, RIGHT_BACK]
        self.gpios = self.initialize_gpio_lines(self.left_motor_pins + self.right_motor_pins)

        # Subscriptions
        self.pbvs_subscription = self.create_subscription(
            String, '/pbvs_data', self.pbvs_callback, 10)
        
        # State variables
        self.target_visible = False
        self.translation_error_x = None
        self.translation_error_y = None
        self.translation_error_z = None
        self.rotation_error_roll = None
        self.rotation_error_pitch = None
        self.rotation_error_yaw = None

    def initialize_gpio_lines(self, pin_pairs):
        """Initialize GPIO lines and set direction to output."""
        gpios = {}
        for pin1, pin2 in pin_pairs:
            line1 = self.chip.get_line(pin1)
            line2 = self.chip.get_line(pin2)
            line1.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)
            line2.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)
            gpios[(pin1, pin2)] = (line1, line2)
        return gpios

    def pbvs_callback(self, msg):
        # Parse the String message data
        data = msg.data

        try:
            # Extract marker ID and errors from the message
            marker_id = int(re.search(r"Marker ID: (\d+)", data).group(1))
            x_error = float(re.search(r"x:\s*(-?\d+\.?\d*)", data).group(1))
            y_error = float(re.search(r"y:\s*(-?\d+\.?\d*)", data).group(1))
            z_error = float(re.search(r"z:\s*(-?\d+\.?\d*)", data).group(1))
            roll_error = float(re.search(r"Roll:\s*(-?\d+\.?\d*)", data).group(1))
            pitch_error = float(re.search(r"Pitch:\s*(-?\d+\.?\d*)", data).group(1))
            yaw_error = float(re.search(r"Yaw:\s*(-?\d+\.?\d*)", data).group(1))

            # Check if the detected marker is the target marker
            if str(marker_id) == TARGET_MARKER_ID:
                self.target_visible = True
                self.translation_error_x = x_error
                self.translation_error_y = y_error
                self.translation_error_z = z_error
                self.rotation_error_roll = roll_error
                self.rotation_error_pitch = pitch_error
                self.rotation_error_yaw = yaw_error
            else:
                self.target_visible = False

            # Control the wheels based on parsed values
            self.control_wheels()

        except AttributeError:
            # Handle parsing errors
            self.get_logger().error('Failed to parse message data. Check format.')

    def control_wheels(self):
        if not self.target_visible:
            # Rotate to search for the target marker if it's not visible
            self.rotate_in_place()
            return

        # Calculate the movement based on translation and rotation errors
        # Step 1: Adjust orientation to align with marker
        if abs(self.rotation_error_yaw) > ORIENTATION_THRESHOLD:
            # Rotate clockwise or counterclockwise to face the marker
            self.adjust_rotation(clockwise=self.rotation_error_yaw > 0)
        
        # Step 2: Translation adjustment
        elif abs(self.translation_error_x) > DISTANCE_THRESHOLD:
            # Use translation errors to adjust forward or backward movement
            if self.translation_error_x > 0:
                self.move_forward_or_backward(forward=True)
            else:
                self.move_forward_or_backward(forward=False)
        
        # Step 3: Fine adjustments in position and orientation if close to marker
        else:
            # Stop the motors when properly aligned and within the threshold
            self.stop_motors()

    def rotate_in_place(self):
        # Rotate in place until the marker comes into view
        self.get_logger().info('Rotating in place to find marker')
        for pin_pair in self.left_motor_pins:
            self.set_motor(pin_pair, forward=True)  # Left wheels forward
        for pin_pair in self.right_motor_pins:
            self.set_motor(pin_pair, forward=False)  # Right wheels backward

    def adjust_rotation(self, clockwise):
        # Rotate to adjust orientation
        if clockwise:
            self.get_logger().info('Adjusting rotation: rotating clockwise')
            for pin_pair in self.left_motor_pins:
                self.set_motor(pin_pair, forward=True)  # Left wheels forward
            for pin_pair in self.right_motor_pins:
                self.set_motor(pin_pair, forward=False)  # Right wheels backward
        else:
            self.get_logger().info('Adjusting rotation: rotating counterclockwise')
            for pin_pair in self.left_motor_pins:
                self.set_motor(pin_pair, forward=False)  # Left wheels backward
            for pin_pair in self.right_motor_pins:
                self.set_motor(pin_pair, forward=True)  # Right wheels forward

    def move_forward_or_backward(self, forward):
        # Move robot forward or backward based on direction
        if forward:
            self.get_logger().info('Moving forward')
        else:
            self.get_logger().info('Moving backward')
        for pin_pair in self.left_motor_pins + self.right_motor_pins:
            self.set_motor(pin_pair, forward)

    def stop_motors(self):
        # Stop all motors
        self.get_logger().info('Stopping motors')
        for pin_pair in self.left_motor_pins + self.right_motor_pins:
            self.set_motor(pin_pair, None)

    def set_motor(self, pin_pair, forward):
        line1, line2 = self.gpios[pin_pair]
        if forward is None:
            # Stop motor
            line1.set_value(0)
            line2.set_value(0)
        elif forward:
            # Move forward
            line1.set_value(1)
            line2.set_value(0)
        else:
            # Move backward
            line1.set_value(0)
            line2.set_value(1)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
