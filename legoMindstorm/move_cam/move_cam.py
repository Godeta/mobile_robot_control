#!/usr/bin/env python3

import time
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_3
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.port import LegoPort
from ev3dev2.display import Display

# Setup Pixy
LegoPort(INPUT_1).mode = 'auto'
time.sleep(2)
pixy = Sensor(INPUT_1)
pixy.mode = 'ALL'

# Motors
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)

# Display & Gyro
lcd = Display()
gyro = GyroSensor(INPUT_3)
gyro.reset()

# Threading flag
move_done = threading.Event()
global move_thread_running
move_thread_running = False

# Instructions: direction time(ms) angle(degrees)
instructions = [
    "forward 11000 0", 
    "right 1000 90", 
    "forward 6000 90",
    "right 1000 180", 
    "forward 0 180", #dynamically updated
    "left 1000 90",
    "forward 3000 90", 
    "backward 3000 90"
]

# Parse instruction
def parse_instruction(instruction):
    parts = instruction.split()
    if len(parts) != 3:
        return None, None, None, None

    direction = parts[0]
    duration = int(parts[1]) / 1000.0  # ms to seconds
    angle = int(parts[2])
    speed = 300

    if direction == "forward":
        return -speed, -speed, duration, angle
    elif direction == "backward":
        return speed, speed, duration, angle
    elif direction == "left":
        return speed, -speed, duration, angle
    elif direction == "right":
        return -speed, speed, duration, angle
    else:
        return 0, 0, 0, angle

# Draw object rectangle from Pixy
def draw_pixy_rect():
    try:
        x = pixy.value(1) * 0.7
        y = pixy.value(2) * 0.6
        w = pixy.value(3) * 0.7
        h = pixy.value(4) * 0.6
        dx, dy = int(w / 2), int(h / 2)
        xa, ya = x - dx, y + dy
        xb, yb = x + dx, y - dy
        lcd.draw.rectangle((xa, ya, xb, yb), fill='black')
    except Exception:
        pass

def move_time(target_angle, left_speed, right_speed, duration):
    """Move the robot for a specified duration with gyro correction"""
    global move_thread_running
    
    # If we're already moving, do nothing
    if move_thread_running:
        return
        
    def _move():
        global move_thread_running
        move_thread_running = True
        move_done.clear()  # Make sure the event is cleared when we start
        
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
        
        # Signal that the move is complete
        move_done.set()
        move_thread_running = False
        
    # Start movement thread
    threading.Thread(target=_move).start()

# Main loop
if __name__ == "__main__":
    step = 0
    total_steps = len(instructions)

    left_motor.reset()
    right_motor.reset()
    gyro.reset()
    
    # Clear move_done initially
    move_done.clear()
    
    # Start first movement
    if instructions:
        ls, rs, dur, angle = parse_instruction(instructions[0])
        move_time(angle, ls, rs, dur)
    
    moving_time = 0
    
    while True:
        lcd.clear()

        # Display info
        if step < total_steps:
            direction, duration_ms, angle = instructions[step].split()
            lcd.draw.text((0, 0), "Step: {}/{}".format(step + 1, total_steps))
            lcd.draw.text((0, 20), "Cmd: {} {} ms".format(direction, duration_ms))
            # lcd.draw.text((0, 40), "Gyro: {:.1f}°".format(gyro.angle))
            lcd.draw.text((0, 60), "Target: {}°".format(angle))
            lcd.draw.text((0, 80), "Move complete: {}".format("Yes" if move_done.is_set() else "No"))

        draw_pixy_rect()
        lcd.update()
        
        # Detect signature for step 4
        if step == 4:
            sig = pixy.value(0)
            if sig == 1:
                moving_time = 4000
            elif sig == 2:
                moving_time = 5000
            elif sig == 3:
                moving_time = 6000
            else:
                moving_time = 3000  # default fallback
            # Update instruction 4 dynamically
            instructions[step] = "forward {} 90".format(moving_time)
        
        # When move is complete AND we're not currently running a movement,
        # advance to the next step and start the next movement
        if move_done.is_set() and not move_thread_running:
            print("Step {} done".format(step))
            step += 1
            
            # Reset the event for the next movement
            move_done.clear()
            
            # If we've completed all steps, break out of the loop
            if step >= total_steps:
                break
                
            # Start the next movement
            ls, rs, dur, angle = parse_instruction(instructions[step])
            move_time(angle, ls, rs, dur)
            
            # Short delay to ensure the thread starts properly
            time.sleep(0.1)
        else:
            # If we haven't started a movement yet (first step) or 
            # if the movement is done but thread cleanup is still happening,
            # wait a short time
            time.sleep(0.05)
            
            # If we're on a step but no movement is running, start it
            if not move_thread_running and step < total_steps:
                ls, rs, dur, angle = parse_instruction(instructions[step])
                move_time(angle, ls, rs, dur)

    # Program complete
    lcd.clear()
    lcd.draw.text((0, 40), "Program Complete")
    lcd.update()
    time.sleep(3)
    
        # diagonale à droite 5s 45°, rota replacer à 0° comme position de base,forward 5s ,gauche rota -90 + forward 3s, couleur detec, gauche rota -180 et movingTime selon la couleur, gauche aligné -270° à la zone de dépot 8s, dépose recul zone 3s même angle, gauche bord 5s -360°, gauche rota -90°, forward prise pièce 5s puis répétition couleur detect,