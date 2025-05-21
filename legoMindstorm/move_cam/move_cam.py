#!/usr/bin/env python3

import time
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_3
from ev3dev2.port import LegoPort
from ev3dev2.display import Display
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.motor import SpeedDPS

# Setup Pixy camera
LegoPort(INPUT_1).mode = 'auto'
time.sleep(2)
pixy = Sensor(INPUT_1)
pixy.mode = 'ALL'

# Display
lcd = Display()

# Gyro setup (assuming it's on INPUT_3)
gyro = GyroSensor(INPUT_3)
gyro.reset()
time.sleep(0.5)  # Give time to reset

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

def move_time(target_angle, left_speed, right_speed, duration):
    """Move the robot for a specified duration with gyro correction"""
    def _move():
        kp = 2.0  # Proportional gain; tune this value based on testing

        start_time = time.time()
        while time.time() - start_time < duration:
            current_angle = gyro.angle
            correction = kp * (current_angle - target_angle)

            # Apply correction (reduce speed on the side it's drifting toward)
            corrected_left_speed = left_speed + correction
            corrected_right_speed = right_speed - correction

            left_motor.on(SpeedDPS(corrected_left_speed))
            right_motor.on(SpeedDPS(corrected_right_speed))
            time.sleep(0.05)  # Short delay to allow smooth updates

        left_motor.off()
        right_motor.off()
        move_done.set()

    move_done.clear()
    threading.Thread(target=_move).start()

if __name__ == "__main__":
    step = 0  # 0 = forward, 1 = left, 2 = done
    move_time(0,-300, -300, 11)  # Move forward with straight angle 11s, 135cm
    
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
                move_time(90, -150, 150, 1)  # Turn right for 1s
                if (current_signature == 1):
                    movingTime = 4
                elif (current_signature == 2):
                    movingTime = 5
                elif (current_signature == 3):
                    movingTime = 6
                step += 1
            elif step == 1:
                lcd.text_pixels("STEP 1", False, 0, 0)
                lcd.update()
                move_time(90,-300, -300, 6)  # Move forward 
                step += 1  # done
            elif step == 2:
                lcd.text_pixels("STEP 2", False, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                move_time(180, -150, 150, 1)  # Turn right for 1s
                step += 1  # done
            elif step == 3:
                lcd.text_pixels("STEP 3", False, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                move_time(180,-300, -300, movingTime)  # forward depending on color
                step += 1  # done
            elif step == 4: 
                lcd.text_pixels("STEP 4", False, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                move_time(90, 150, -150, 1)  # Turn left for 1s
                step += 1  # done
            elif step ==5:
                lcd.text_pixels("STEP 5", False, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                move_time(90,-300, -300, 3)  # forward depending on color
                step += 1  # done
            elif step == 6:
                lcd.text_pixels("STEP 6 - END", False, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                move_time(90,150, 150, 2)  # back for 2s
                step += 1  # done
                break  # Do nothing more

        time.sleep(0.05)

        # diagonale à droite 5s, rota 0°,forward 5s, gauche rota + forward, couleur detec, gauche rota et moveColor s, gauche aligné chute 8s, dépose recul zone avant 3s, gauche bord 5s, gauche rota et prise 5s,