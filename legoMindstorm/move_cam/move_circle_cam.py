#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveDifferential, LargeMotor, SpeedRPM, SpeedDPS
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_3
from ev3dev2.display import Display
from ev3dev2.wheel import EV3Tire
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import GyroSensor
import time

# Setup Pixy camera
LegoPort(INPUT_1).mode = 'auto'
time.sleep(2)
pixy = Sensor(INPUT_1)
pixy.mode = 'ALL'

# https://www.brickowl.com/catalog/lego-mindstorms-ev3-set-31313/inventory pièce 44309 doc code https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html#large-ev3-motor
# Initialize the display
lcd = Display()
lcd.clear()
lcd.text_pixels("Robot Navigation Program", False, 0, 0)
lcd.update()

# Gyro setup (assuming it's on INPUT_3)
gyro = GyroSensor(INPUT_3)
gyro.reset()
time.sleep(0.5)  # Give time to reset

# Variable to track the detected signature
current_signature = 0
saved_signature = 0
is_scanning = False  # Flag to control when camera is actively scanning

# Motors
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)
# Flag to signal move_time completion
# move_done = threading.Event()

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
    # def _move():
    kp = 4.0  # Proportional gain; tune this value based on testing

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
    # move_done.set()

    # move_done.clear()
    # threading.Thread(target=_move).start()

# Set up the robot parameters
# - Uses standard lego wheels
# - Wheels are 125mm apart
wheel_distance_mm = 125

try:
    # Create MoveDifferential instance
    robot = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, wheel_distance_mm)
    
    # Start odometry to track position
    lcd.text_pixels("Starting odometry...", False, 0, 10)
    lcd.update()
    robot.odometry_start()
    time.sleep(1)  # Give time for odometry to initialize
    
    # Move to coordinates (0, 500) - 850cm forward
    segment_length_mm = 780
    arc_radius = 500
    perimeter = 750
    lcd.text_pixels("Moving to (0, 800)...", False, 0, 20)
    lcd.update()
    robot.on_to_coordinates(SpeedRPM(-50), -180, segment_length_mm)
    
    # Wait a moment after reaching the position
    time.sleep(1)
    
    # Drive in arc to the right along an imaginary circle of radius 300 mm
    # Drive for 200 mm around this imaginary circle
    lcd.text_pixels("Driving in arc...", False, 0, 30)
    lcd.update()

    # stop avant et détection couleur
    inc_dist = 250
    add_inc_dist = inc_dist/10
    cumul = 0
    insideDetect = False # vrai si on est passé à une valeur autre que 0 puis de retour à 0
    robot.on_arc_right(SpeedRPM(-40), arc_radius, perimeter-inc_dist)
    has_seen_object = False
    while cumul < inc_dist and not insideDetect:
        cumul += add_inc_dist
        #check cam signature
        current_signature = get_signature()
        if current_signature != 0 and has_seen_object == False:
                saved_signature = current_signature
                has_seen_object = True
        elif current_signature != saved_signature :
                insideDetect = True
        
        lcd.text_pixels("Signature: ", True, 0, 0)
        lcd.text_pixels(str(saved_signature), False, 89, 34)
        lcd.text_pixels("Inside detect : ", False, 89, 48)
        lcd.text_pixels(str(insideDetect), False, 89, 60)
        lcd.update()
        # go forward
        robot.on_arc_right(SpeedRPM(-40), arc_radius, add_inc_dist)
    
    
    # Rotate to angular position 90 degrees clockwise
    # robot.turn_to_angle(SpeedRPM(-40), 90)
    move_time(180, -150, 150, 1200)  # Turn right for 1s (1000 ms)

    # distance depending on save sign value
    if saved_signature == 1:
        dist = 300
    elif saved_signature == 2:
        dist = 300 + 270
    elif saved_signature == 3:
        dist = 570 + 270
    else:
        dist = 10
    
    # straight forward
    robot.on_for_distance(SpeedRPM(-40), dist)

    # robot.robot.turn_to_angle(SpeedRPM(-40), -90)
    move_time(90, 150, -150, 1100)  # Turn left for 1s (1000 ms

    robot.on_for_distance(SpeedRPM(-40), 850-cumul/5)

    # Wait a moment after completing the arc
    time.sleep(1)
    
    # Display final position from odometry
    x, y = robot.x_pos_mm, robot.y_pos_mm
    lcd.text_pixels("Final: x={:.1f}, y={:.1f}".format(x, y), False, 0, 40)
    # lcd.update()
    
    # Stop odometry
    robot.odometry_stop()
    lcd.text_pixels("Odometry stopped. Program complete.", False, 0, 50)
    lcd.update()
    
except Exception as e:
    # Display any errors
    lcd.clear()
    lcd.text_pixels("Error: {}".format(str(e)), False, 0, 0)
    print("Error: {}".format(str(e)))
    lcd.update()

# Wait before ending the program
time.sleep(5)