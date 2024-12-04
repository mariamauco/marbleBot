import gpiod

# Define GPIO chip and lines for motor control
CHIP_NAME = "/dev/gpiochip0"  # Default GPIO chip
LEFT_FRONT = (23, 24)  # GPIO pins for left front motor (IN1, IN2)
LEFT_BACK = (12, 25)   # GPIO pins for left back motor (IN1, IN2)
RIGHT_FRONT = (16, 27) # GPIO pins for right front motor (IN1, IN2)
RIGHT_BACK = (17, 22)   # GPIO pins for right back motor (IN1, IN2)

MOTOR_PINS = {
    'left_front': LEFT_FRONT,
    'left_back': LEFT_BACK,
    'right_front': RIGHT_FRONT,
    'right_back': RIGHT_BACK
}

def initialize_gpio_lines(chip, motor_pins):
    """Initialize GPIO lines for all motor pins."""
    gpios = {}
    for motor, pins in motor_pins.items():
        in1, in2 = pins
        line1 = chip.get_line(in1)
        line2 = chip.get_line(in2)
        line1.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)
        line2.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)
        gpios[motor] = (line1, line2)
    return gpios

def set_motor(lines, forward):
    """Set motor direction."""
    line1, line2 = lines
    if forward == "forward":
        line1.set_value(1)
        line2.set_value(0)
    elif forward == "backward":
        line1.set_value(0)
        line2.set_value(1)
    else:  # Stop
        line1.set_value(0)
        line2.set_value(0)

def main():
    # Initialize GPIO
    chip = gpiod.Chip('gpiochip4')
    motor_lines = initialize_gpio_lines(chip, MOTOR_PINS)

    try:
        while True:
            print("Select motor to control (left_front, left_back, right_front, right_back):")
            motor = input("Motor: ").strip().lower()

            if motor not in MOTOR_PINS:
                print("Invalid motor name. Try again.")
                continue

            print("Enter direction (forward, backward, stop):")
            direction = input("Direction: ").strip().lower()

            if direction not in ["forward", "backward", "stop"]:
                print("Invalid direction. Try again.")
                continue

            # Control the selected motor
            set_motor(motor_lines[motor], direction)
            print(f"{motor} is running {direction}.")

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        # Ensure all motors are stopped when the program exits
        for lines in motor_lines.values():
            set_motor(lines, "stop")

if __name__ == "__main__":
    main()
