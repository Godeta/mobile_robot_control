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
saved_signature = 0
is_scanning = False  # Flag to control when camera is actively scanning

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

def move_time(target_angle, left_speed, right_speed, duration_ms):
    """
    Move the robot for a specified duration with gyro correction
    duration_ms: duration in milliseconds
    """
    def _move():
        kp = 2.0  # Proportional gain; tune this value based on testing

        start_time = time.time()
        duration_sec = duration_ms / 1000.0  # Convert milliseconds to seconds
        while time.time() - start_time < duration_sec:
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
    step = 0  # Keep track of movement sequence
    has_seen_line = Falses
    
    # Initial message
    lcd.text_pixels("Searching for objects...", True, 0, 0)
    lcd.update()
    
    # Step 0: Move forward for 9.5 seconds
    move_time(0, 400, 400, 8500)  # 9.5 seconds = 9500 ms

    while True:
        # Handle camera scanning only during specific steps
        if is_scanning:
            # Get current signature only if we haven't saved one yet
            if saved_signature == 0:
                current_signature = get_signature()
            
            # Update saved_signature if a valid signature is detected
            if current_signature != 0:
                saved_signature = current_signature
                print("Signature detected: ", saved_signature)
            else:
                current_signature = saved_signature
            
        # Handle movement sequence
        if move_done.is_set():
            if step == 0:
                # Step 3: Check for signature and proceed
                lcd.text_pixels("STEP 2: Forward small", True, 0, 0)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                # Start camera scanning during diagonal movement
                is_scanning = True
                move_time(0, -100, -100, 800)
                step += 1
                
            elif step == 1:
                # Turn left
                lcd.text_pixels("STEP 2-2: Turn left", True, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.update()
                move_time(-90, 150, -150, 1400)
                step += 1
            elif step == 2:
                # Step 3: Check for signature and proceed
                lcd.text_pixels("STEP 3: Checking signature", True, 0, 0)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.update()
                
                # Set movingTime based on detected signature
                if saved_signature == 1:
                    movingTime = 3000  # 4 seconds = 4000 ms
                elif saved_signature == 2:
                    movingTime = 5400  # 5 seconds = 5000 ms
                elif saved_signature == 3:
                    movingTime = 6700  # 6 seconds = 6000 ms
                else:
                    movingTime = 5400  # Default if no signature detected
                
                # Continue with original sequence: Forward 6s
                move_time(-90, -300, -300, 7000)
                step += 1
            elif step == 3:
                # Turn right
                lcd.text_pixels("STEP 4: Turn right", True, 0, 0)
                lcd.text_pixels(str(gyro.angle), False, 89, 64)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.update()
                move_time(0, -100, 150, 1000)
                step += 1
            elif step == 4:
                # Forward depending on color
                lcd.text_pixels("STEP 5: Forward by color", True, 0, 0)
                lcd.text_pixels(str(gyro.angle), True, 89, 64)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.update()
                move_time(0, -300, -300, movingTime)
                step += 1
            elif step == 5:
                # Turn left
                lcd.text_pixels("STEP 6: Turn left", True, 0, 0)
                lcd.text_pixels(str(gyro.angle), True, 89, 64)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.update()
                move_time(-90, 150, -150, 1000)
                step += 1
            elif step == 6:
                # Forward
                lcd.text_pixels("STEP 7: Forward", True, 0, 0)
                lcd.text_pixels(str(saved_signature), False, 89, 64)
                lcd.text_pixels(str(gyro.angle), False, 89, 34)
                lcd.update()
                move_time(-90, -300, -300, 1500)
                step += 1
            elif step == 7:
                lcd.text_pixels("STEP 8: Loop depot", True, 0, 0)
                lcd.text_pixels(str(gyro.angle), True, 89, 64)
                lcd.text_pixels(str(saved_signature), False, 89, 34)
                lcd.update()
                # loop go forward slowly, see If there is a color, then go forward a specific distance when you see It
                while has_seen_line == False:
                    move_time(-90, -300, -300, 150)
                    #check cam signature
                    current_signature = get_signature()
                    if current_signature != 0:
                        has_seen_line = True
                move_time(-90, -300, -300, 1000)
                step += 1

            elif step == 7:
                # Backward
                lcd.text_pixels("STEP 8: Backward - END", True, 0, 0)
                lcd.text_pixels(str(saved_signature), False, 89, 64)
                lcd.update()
                move_time(-90, 150, 150, 5000)
                step += 1
                break  # End the program

        time.sleep(0.05)

    while True:
        lcd.text_pixels(" signature", True, 0, 0)
        lcd.text_pixels(str(saved_signature), False, 89, 64)
        lcd.update()