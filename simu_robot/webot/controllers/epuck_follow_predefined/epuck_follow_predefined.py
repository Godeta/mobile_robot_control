"""follow predefined"""
# Program to execute a table of instructions using the unicycle model
# needs to implement the kalman filter

from controller import Supervisor, Robot, DistanceSensor, Motor
from unicycle_model import *
import numpy as np
import math
import sys
def main():
    # Initialize variables
    x = 0           # position in x [m]
    y = -0.43       # position in y [m]
    phi = 0         # orientation [rad]
    dx = 0          # speed in x [m/s]
    dy = 0          # speed in y [m/s]
    dphi = 0        # orientation speed [rad/s]
    wl = 0          # angular speed of the left wheel [rad/s]
    wr = 0          # angular speed of the right wheel [rad/s]
    u = 0           # linear speed [m/s]
    w = 0           # angular speed [rad/s]

    TIME_STEP = 64
    MAX_SPEED = 6.28

    # Robot physical parameters
    R = 0.0205      # radius of the wheels [m]: 20.5mm 
    D = 0.052       # distance between the wheels [m]: 52mm 

    # create the Robot instance
    robot = Supervisor()

    # get the time step of the current world
    timestep = int(robot.getBasicTimeStep())   # [ms]

    # Constants for instruction execution
    FORWARD_SPEED = 0.05                      # [m/s]
    ROTATE_SPEED = 0.5                        # [rad/s]
    COMPLETION_THRESHOLD = 0.01               # threshold to consider an instruction complete

    # Define the instruction types
    ROTATE = "ROTATE"
    FORWARD = "FORWARD"

    # Instructions and weights (example)
    # instructions = [ROTATE, FORWARD, ROTATE, FORWARD]
    # weights = [90, 20, -45, 30]  # 90 degrees, 20 cm, -45 degrees, 30 cm
    args = sys.argv
    val = 0
    if len(args) >0 :
        val = int(args[1])
    if val == 1:
        # You can change these instructions as needed
        instructions = [FORWARD, ROTATE, FORWARD, ROTATE, FORWARD, ROTATE, FORWARD, ROTATE, FORWARD]
        weights = [20, 180, 20, 90, 40, -90, 20, 180, 30]  # degrees or centimeters
    else:
        # You can change these instructions as needed
        instructions = [FORWARD, ROTATE, FORWARD, ROTATE, FORWARD, ROTATE, FORWARD]
        weights = [30, -90, 5, 180, 5, 90, 30]  # degrees or centimeters
    print(val)
    print(args)
    #-------------------------------------------------------
    # Initialize devices

    # encoder
    encoder = []
    encoderNames = ['left wheel sensor', 'right wheel sensor']
    for i in range(2):
        encoder.append(robot.getDevice(encoderNames[i]))
        encoder[i].enable(timestep)

    # motors    
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    # State machine variables
    current_instruction_index = 0
    instruction_state = "NOT_STARTED"
    target_value = 0
    accumulated_value = 0
    start_x, start_y, start_phi = x, y, phi

    print("Starting instruction execution program!")
    print(f"Instructions: {instructions}")
    print(f"Weights: {weights}")

    oldEncoderValues = [0, 0]
    x_old = x
    y_old = y    
    phi_old = phi

    # For tracking rotation
    total_rotation = 0  # Track total rotation for ROTATE instructions

    while robot.step(timestep) != -1:
        # Read encoder values
        encoderValues = []
        for i in range(2):
            encoderValues.append(encoder[i].getValue())    # [rad]
        
        # Update odometry -> needs to be precise otherwise all the controls are wrong !
        u, w = get_robot_speeds(leftMotor.getVelocity(), rightMotor.getVelocity(), R, D)
        x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, timestep/1000)
        
        # State machine for executing instructions
        if current_instruction_index < len(instructions):
            # Get current instruction and weight
            instruction = instructions[current_instruction_index]
            weight = weights[current_instruction_index]
            
            if instruction_state == "NOT_STARTED":
                # Reset accumulated value and record start position
                accumulated_value = 0
                total_rotation = 0
                start_x, start_y, start_phi = x, y, phi
                
                # Convert weight to target value based on instruction type
                if instruction == ROTATE:
                    # Convert degrees to radians
                    target_value = math.radians(weight)
                    print(f"Starting ROTATE: {weight} degrees ({target_value:.4f} radians)")
                elif instruction == FORWARD:
                    # Convert centimeters to meters
                    target_value = weight / 100.0
                    print(f"Starting FORWARD: {weight} cm ({target_value:.4f} m)")
                
                instruction_state = "IN_PROGRESS"
            
            elif instruction_state == "IN_PROGRESS":
                if instruction == ROTATE:
                    # Calculate desired angular speed (positive or negative based on target)
                    desired_w = ROTATE_SPEED if target_value > 0 else -ROTATE_SPEED
                    desired_u = 0  # No forward movement during rotation
                    
                    # Calculate wheel speeds from desired robot speeds
                    wl_d, wr_d = wheel_speed_commands(desired_u, desired_w, D, R)
                    
                    # Set motor speeds
                    leftMotor.setVelocity(wl_d)
                    rightMotor.setVelocity(wr_d)
                    
                    # Update accumulated rotation
                    # Calculate the change in orientation (phi)
                    delta_phi = phi - phi_old
                    
                    # Handle wrap-around at +/- pi
                    if delta_phi > math.pi:
                        delta_phi -= 2 * math.pi
                    elif delta_phi < -math.pi:
                        delta_phi += 2 * math.pi
                    
                    # Add to the total rotation
                    total_rotation += delta_phi
                    
                    # Calculate progress - using the absolute value for progress display
                    if target_value != 0:
                        progress = (abs(total_rotation) / abs(target_value)) * 100
                    else:
                        progress = 100
                    
                    # Check if rotation is complete
                    if (target_value > 0 and total_rotation >= target_value) or \
                    (target_value < 0 and total_rotation <= target_value):
                        print(f"Completed ROTATE: accumulated {math.degrees(total_rotation):.2f} degrees")
                        leftMotor.setVelocity(0)
                        rightMotor.setVelocity(0)
                        instruction_state = "COMPLETED"
                    else:
                        print(f"Position (x, y, phi): ({x:.3f}m, {y:.3f}m, {math.degrees(phi):.2f}°), "
                            f"Progress: {progress:.2f}%, Accumulated: {math.degrees(total_rotation):.2f}°")
                
                elif instruction == FORWARD:
                    # Calculate desired speeds for forward movement
                    desired_u = FORWARD_SPEED
                    desired_w = 0  # No rotation during forward movement
                    
                    # Calculate wheel speeds from desired robot speeds
                    wl_d, wr_d = wheel_speed_commands(desired_u, desired_w, D, R)
                    
                    # Set motor speeds
                    leftMotor.setVelocity(wl_d)
                    rightMotor.setVelocity(wr_d)
                    
                    # Calculate distance moved
                    distance = math.sqrt((x - start_x)**2 + (y - start_y)**2)
                    accumulated_value = distance
                    
                    # Calculate progress
                    if target_value != 0:
                        progress = (accumulated_value / target_value) * 100
                    else:
                        progress = 100
                    
                    # Check if forward movement is complete
                    if accumulated_value >= target_value:
                        print(f"Completed FORWARD: moved {accumulated_value*100:.2f} cm")
                        leftMotor.setVelocity(0)
                        rightMotor.setVelocity(0)
                        instruction_state = "COMPLETED"
                    else:
                        print(f"Position (x, y, phi): ({x:.3f}m, {y:.3f}m, {math.degrees(phi):.2f}°), "
                            f"Progress: {progress:.2f}%, Moved: {accumulated_value*100:.2f}cm")
            
            elif instruction_state == "COMPLETED":
                # Move to the next instruction
                current_instruction_index += 1
                instruction_state = "NOT_STARTED"
                
                # Brief pause between instructions
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
        
        else:
            # All instructions completed
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            if instruction_state != "ALL_DONE":
                print("All instructions completed!")
                instruction_state = "ALL_DONE"
        
        # Update old values for next iteration
        oldEncoderValues = encoderValues
        x_old = x
        y_old = y
        phi_old = phi

if __name__ == "__main__":
    main()