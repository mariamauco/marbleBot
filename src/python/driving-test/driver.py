import gpiod
import os

import sshkeyboard

def press(key):
    print(f"Key pressed: {key}")
    
    

sshkeyboard.listen_keyboard(on_press=press)
