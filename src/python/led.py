import gpiod
import time

INPUT_PIN = 14
LED_PIN = 14
chip = gpiod.Chip('gpiochip4')

led_line = chip.get_line(LED_PIN)
led_line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)

try:
    while True:
        led_line.set_value(1)
        time.sleep(1)
        led_line.set_value(0)
        time.sleep(1)
finally:
    led_line.set_value(0)
    led_line.release()