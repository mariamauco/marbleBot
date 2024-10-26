import gpiod

INPUT_PIN = 14
chip = gpiod.Chip('gpiochip4')
input_line = chip.get_line(INPUT_PIN)
input_line.request(consumer="Button", type=gpiod.LINE_REQ_DIR_IN)
try:
   while True:
       inputs = input_line.get_value()
       print(inputs)
       
finally:
    input_line.release()