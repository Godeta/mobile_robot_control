#!/usr/bin/env python3

import time
from ev3dev2.sensor import Sensor, INPUT_1
from ev3dev2.port import LegoPort
from ev3dev2.display import Display

# Setup Pixy camera
LegoPort(INPUT_1).mode = 'auto'
print("Initializing camera...")
time.sleep(2)
pixy = Sensor(INPUT_1)
pixy.mode = 'ALL'

# Setup display
lcd = Display()

# Function to get signature
def get_signature():
    try:
        signature = pixy.value(0)
        if signature in [1, 2, 3]:
            return signature
        return 0
    except:
        return 0

# Main program
while True:
    # Get current signature
    current_signature = get_signature()
    
    # Update display
    lcd.clear()
    lcd.text_pixels("Signature:", True, 0, 0)
    lcd.text_pixels(str(current_signature), False, 89, 64)
    lcd.update()
    
    # Print to console
    print("Signature: {}".format(current_signature))
    
    # Wait 1 second
    time.sleep(1)