#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveDifferential, SpeedRPM
from ev3dev2.display import Display
from ev3dev2.wheel import EV3Tire
import time

# https://www.brickowl.com/catalog/lego-mindstorms-ev3-set-31313/inventory pièce 44309 doc code https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html#large-ev3-motor
# Initialize the display
lcd = Display()
lcd.clear()
lcd.text_pixels("Robot Navigation Program", False, 0, 0)
lcd.update()

# Set up the robot parameters
# - Uses custom 40mm diameter wheels
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
    robot.on_to_coordinates(SpeedRPM(-50), -140, segment_length_mm)
    
    # Wait a moment after reaching the position
    time.sleep(1)
    
    # Drive in arc to the right along an imaginary circle of radius 300 mm
    # Drive for 200 mm around this imaginary circle
    lcd.text_pixels("Driving in arc...", False, 0, 30)
    lcd.update()
    robot.on_arc_right(SpeedRPM(-40), arc_radius, perimeter)
    # robot.on_arc_right(SpeedRPM(-40), arc_radius, perimeter/2)
    # # stop avant et détection couleur, si j'en vois une puis que je la perds de vue alors je l'ai, je tourne
    # robot.on_arc_right(SpeedRPM(-40), arc_radius, perimeter/4)
    # robot.on_arc_right(SpeedRPM(-40), arc_radius, perimeter/4)
    
    # Rotate 90 degrees clockwise
    robot.turn_right(SpeedRPM(-40), 90)
    # straight forward
    robot.on_for_distance(SpeedRPM(-40), 100)

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