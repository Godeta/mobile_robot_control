#!/usr/bin/env python3

import time
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
from ev3dev2.sensor import Sensor, INPUT_1
from ev3dev2.port import LegoPort
from ev3dev2.display import Display

# Setup Pixy camera
LegoPort(INPUT_1).mode = 'auto'
time.sleep(2)
pixy = Sensor(INPUT_1)
pixy.mode = 'ALL'

# Display
lcd = Display()

# Motors
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)
# Flag to signal move_time completion
move_done = threading.Event()

# Variable to track the detected signature
current_signature = 0
last_reported_signature = 0
last_report_time = 0

def get_signature():
    """
    Get the current signature detected by Pixy camera
    Returns signature number (1-3) or 0 if none detected
    """
    try:
        # In 'ALL' mode, the first value indicates signature number
        signature = pixy.value(0)
        
        # Check if it's one of our target signatures (1, 2, or 3)
        if signature in [1, 2, 3]:
            return signature
        return 0
    except Exception:
        return 0

def draw_pixy_rect():
    """Draw a rectangle on the EV3 display showing what Pixy sees"""
    try:
        signature = pixy.value(0)
        if signature > 0:
            x = pixy.value(1) * 0.7
            y = pixy.value(2) * 0.6
            w = pixy.value(3) * 0.7
            h = pixy.value(4) * 0.6
            dx, dy = int(w / 2), int(h / 2)
            xa, ya = x - dx, y + dy
            xb, yb = x + dx, y - dy

            lcd.clear()
            # Display the signature number
            lcd.text_pixels(signature, False, 10, 10)
            lcd.draw.rectangle((xa, ya, xb, yb), fill='black')
            lcd.update()
    except Exception:
        pass  # If Pixy returns invalid values

def move_time(left_speed, right_speed, duration):
    """Move the robot for a specified duration"""
    def _move():
        left_motor.on(SpeedDPS(left_speed))
        right_motor.on(SpeedDPS(right_speed))
        time.sleep(duration)
        left_motor.off()
        right_motor.off()
        move_done.set()

    move_done.clear()
    threading.Thread(target=_move).start()

if __name__ == "__main__":
    step = 0  # 0 = forward, 1 = left, 2 = done
    move_time(-300, -300, 2)  # Move forward 2s, 17cm
    
    # Initial message
    lcd.text_pixels("Searching for objects...", False, 0, 0)
    lcd.update()

    while True:
        # Get current signature
        current_signature = get_signature()
        
        # Report the signature when it changes or if it's been a while
        current_time = time.time()
        if (current_signature != 0 and 
            (current_signature != last_reported_signature or current_time - last_report_time > 2)):
            print("Signature : ")
            print(current_signature)
            last_reported_signature = current_signature
            last_report_time = current_time
            
        # Draw what Pixy sees
        draw_pixy_rect()

        # Handle movement sequence
        if move_done.is_set():
            if step == 0:
                move_time(-150, 150, 1)  # Turn left for 1s
                step += 1
            elif step == 1:
                step += 1  # done
            elif step == 2:
                pass  # Do nothing more

        time.sleep(0.05)
# https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:lego_chase_demo et https://github.com/KWSmit/pixy2_pybricks