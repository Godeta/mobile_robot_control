"""line_following_behavior controller."""
# Use this as a template to implement the line-following state-machine

from controller import Robot, DistanceSensor, Motor
from unicycle_model import *
import numpy as np
print("Starting program !")
#-------------------------------------------------------
# Initialize variables
x = 0 #position in x [m]
y = -0.43 #position in y [m]
phi = 0 #orientation [rad]
dx = 0 # speed in x [m/s]
dy = 0 # speed in y [m/s]
dphi = 0 #orientation speed [rad/s]
wl = 0 #angular speed of the left wheel [rad/s]
wr = 0 #angular speed of the right wheel [rad/s]
u = 0 #linear speed [m/s]
w = 0 #angular speed [rad/s]

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
R = 0.0205 # radius of the wheels [m]: 20.5mm 
D = 0.052 # distance between the wheels [m]: 52mm 

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 5

# Define the states
states = ['line_following', 'lost_line', 'turning_left', 'turning_right']
current_state = states[0]  # Initialize the current state to line_following

# Define the sensor thresholds
LINE_THRESHOLD = 350  # adjust this value based on your sensor readings

# Define a variable to switch the sensor logic
INVERT_SENSOR_LOGIC = True  # Set to True to detect line as lower than threshold

# Define the motor velocities
VELOCITY_FORWARD = 0.5 * MAX_SPEED
VELOCITY_TURN = 0.5 * MAX_SPEED

leftSpeed = 0
rightSpeed = 0

# Define a counter to track the number of cycles without seeing the line
lost_line_counter = 0
OBSTACLE_THRESHOLD = 120  # Adjust based on your environment
# Add these initializations with your other variables at the beginning of your program
search_counter = 0
backup_counter = 0
forward_counter = 0
rotate_counter = 0
recovery_direction = 'search'  # Default recovery direction

#-------------------------------------------------------
# Initialize devices

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# Distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

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
leftMotor.setVelocity(6.0)
rightMotor.setVelocity(6.0)

oldEncoderValues = [0, 0]
x_old = x
y_old = y    
phi_old = phi
while robot.step(timestep) != -1:
    # Update sensor readings
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]

    

    print('Counter: ' + str(counter) + '. Current state: ' + current_state + '\nline values' + str(gsValues) + '\nobstacle values' + str(psValues))

    # Check for obstacles first (higher priority)
    front_obstacle = psValues[0] > OBSTACLE_THRESHOLD or psValues[7] > OBSTACLE_THRESHOLD
    left_obstacle = psValues[5] > OBSTACLE_THRESHOLD or psValues[6] > OBSTACLE_THRESHOLD
    right_obstacle = psValues[1] > OBSTACLE_THRESHOLD or psValues[2] > OBSTACLE_THRESHOLD
    
    if front_obstacle and current_state != 'obstacle_avoid':
        # Switch to obstacle avoidance mode
        current_state = 'obstacle_avoid'
        counter = 0

    # State machine logic
    if current_state == 'obstacle_avoid':
        # Wall following behavior
        if front_obstacle:
            # If obstacle in front, turn right to avoid it
            leftSpeed = 0.7 * MAX_SPEED
            rightSpeed = -0.3 * MAX_SPEED
        elif left_obstacle:
            # If obstacle on the left, follow it (wall following)
            leftSpeed = 0.7 * MAX_SPEED
            rightSpeed = 0.3 * MAX_SPEED
        elif right_obstacle:
            # If obstacle on the right but not in front, turn slightly left
            leftSpeed = 0.3 * MAX_SPEED
            rightSpeed = 0.7 * MAX_SPEED
        else:
            # No obstacles detected, increment counter
            counter += 1
            # Continue moving forward while turning slightly to find the line
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = 0.6 * MAX_SPEED
            
            # After some time without obstacles, return to line following
            if counter > COUNTER_MAX:
                current_state = 'line_following'
                counter = 0

    if current_state == 'line_following':
        if gsValues[0] < LINE_THRESHOLD and gsValues[2] < LINE_THRESHOLD:
            # both sensors see the line, move forward
            leftSpeed = VELOCITY_FORWARD
            rightSpeed = VELOCITY_FORWARD
            lost_line_counter = 0  # reset the counter
        elif (gsValues[0] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC:
            # left sensor sees the line, turn right
            leftSpeed = 0.8 * MAX_SPEED
            rightSpeed = 0.2 * MAX_SPEED
            turn_counter = 0  # reset turn counter when starting a turn
            current_state = 'turning_right'
        elif (gsValues[2] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC:
            # right sensor sees the line, turn left
            leftSpeed = 0.2 * MAX_SPEED
            rightSpeed = 0.8 * MAX_SPEED
            turn_counter = 0  # reset turn counter when starting a turn
            current_state = 'turning_left'
        else:
            # both sensors lost the line, increment the counter
            lost_line_counter += 1
            if lost_line_counter > 2:  # adjust this value to suit your needs
                # both sensors lost the line for too long, start recovery
                current_state = 'lost_line'
                recovery_direction = 'search'  # Start with a search pattern

    elif current_state == 'turning_right':
        # turn right
        leftSpeed = 0.8 * MAX_SPEED
        rightSpeed = 0.1 * MAX_SPEED
        
        # Increment turn counter to prevent getting stuck in turning state
        turn_counter += 1
        
        # Check if both sensors see the line (back on track)
        if gsValues[0] < LINE_THRESHOLD and gsValues[2] < LINE_THRESHOLD:
            current_state = 'line_following'
        # Check if right sensor sees the line (completing the turn)
        elif (gsValues[2] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC:
            current_state = 'line_following'
        # Force return to line following after a certain time to prevent getting stuck
        elif turn_counter > 50:  # Adjust this threshold based on your robot's speed
            current_state = 'line_following'

    elif current_state == 'turning_left':
        # turn left
        leftSpeed = 0.1 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED
        
        # Increment turn counter to prevent getting stuck in turning state
        turn_counter += 1
        
        # Check if both sensors see the line (back on track)
        if gsValues[0] < LINE_THRESHOLD and gsValues[2] < LINE_THRESHOLD:
            current_state = 'line_following'
        # Check if left sensor sees the line (completing the turn)
        elif (gsValues[0] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC:
            current_state = 'line_following'
        # Force return to line following after a certain time to prevent getting stuck
        elif turn_counter > 50:  # Adjust this threshold based on your robot's speed
            current_state = 'line_following'

    elif current_state == 'lost_line':
        # Implement a more sophisticated recovery strategy
        if recovery_direction == 'search':
            # First try rotating in place to find the line
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            search_counter += 1
            
            # Check if either sensor detects the line
            if ((gsValues[0] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC or 
                (gsValues[2] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC):
                # Line found, go back to line following
                current_state = 'line_following'
                
            # If we've been searching for too long, try backing up
            elif search_counter > 100:  # Adjust based on robot speed
                recovery_direction = 'backup'
                backup_counter = 0
                
        elif recovery_direction == 'backup':
            # Back up a bit to try to find the line
            leftSpeed = -0.3 * MAX_SPEED
            rightSpeed = -0.3 * MAX_SPEED
            backup_counter += 1
            
            # Check if we found the line while backing up
            if ((gsValues[0] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC or 
                (gsValues[2] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC):
                current_state = 'line_following'
                
            # If backing up doesn't work, try going forward
            elif backup_counter > 50:  # Adjust based on robot speed
                recovery_direction = 'forward'
                forward_counter = 0
                
        elif recovery_direction == 'forward':
            # Go forward a bit to try to find the line
            leftSpeed = 0.3 * MAX_SPEED
            rightSpeed = 0.3 * MAX_SPEED
            forward_counter += 1
            
            # Check if we found the line
            if ((gsValues[0] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC or 
                (gsValues[2] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC):
                current_state = 'line_following'
                
            # If going forward doesn't work, try a 180° turn (might be at end of line)
            elif forward_counter > 50:  # Adjust based on robot speed
                recovery_direction = 'rotate180'
                rotate_counter = 0
                
        elif recovery_direction == 'rotate180':
            # Do a 180° turn to find the line in the opposite direction
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            rotate_counter += 1
            
            # Check if we found the line during rotation
            if ((gsValues[0] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC or 
                (gsValues[2] > LINE_THRESHOLD) == INVERT_SENSOR_LOGIC):
                current_state = 'line_following'
                
            # After completing approximately 180°, go back to searching
            elif rotate_counter > 150:  # Adjust for your robot's turning speed
                recovery_direction = 'search'
                search_counter = 0

    # Set motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # odometry
    # pulses_per_turn = 72
    # wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)
    u, w = get_robot_speeds(leftSpeed, rightSpeed, R, D)
    x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, timestep/1000)

    # Print the current state
    # print(f"Estimated position (x, y, phi): ({x:.3f}, {y:.3f}, {phi:.3f})" + f"speed (u, w): ({u:.3f}, {w:.3f})")
    # Update the old encoder values for the next iteration
    oldEncoderValues = encoderValues
    x_old = x
    y_old = y
    phi_old = phi

