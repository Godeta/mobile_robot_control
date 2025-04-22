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
states = ['goToGoal', 'obstacle_avoid']
current_state = states[0]  # Initialize the current state to goTOGOAL

# Define the motor velocities
leftSpeed = 0
rightSpeed = 0

# Define a counter to track the number of cycles without seeing the line
OBSTACLE_THRESHOLD = 80  # Adjust based on your environment

#-------------------------------------------------------
# Initialize devices

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
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]

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
                current_state = 'goToGoal'
                counter = 0

    if current_state == 'goToGoal':
        # goal
        xd = -0.5
        yd = 0
        phi_d = 0
        delta_t = (timestep - prev_timestep)/1000 # [s]
        # Position error:
        x_err = xd - x
        y_err = yd - y
        dist_err = np.sqrt(x_err**2 + y_err**2)

        # Orientation error
        phi_d = np.arctan2(y_err,x_err)
        phi_err = phi_d - phi

        # Limit the error to (-pi, pi):
        phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))
        
        # kp = 1
        # ki = 2
        # kd = 1
        # prev_timestep = 0
        # e_acc = 0
        # e_prev = 0
        # de_acc = 0
        # de_prev = 0
        # When implementing this, you must update e_prev and e_acc properly at every step.
        # Controller gains:
        kp = 1
        kd = 0.01
        ki = 0.1

        # Obtain the desired linear and angular speed:
        if (delta_t == 0):
            delta_t = 0.001
        w_d, e_prev, e_acc = pid_controller(phi_err_correct, e_prev, 0, delta_t, kp, kd, ki) #PID algortithm: must be executed every delta_t seconds
        u_d, de_prev, de_acc = pid_controller(dist_err, de_prev, 0, delta_t, kp, kd, ki)
        
        # robot command
        leftSpeed, rightSpeed = wheel_speed_commands(u_d, w_d, D, R)
        # must
    if current_state == 'stop':
        # stop the robot
        leftSpeed = 0
        rightSpeed = 0

    # Set motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    print("u_d, w_d, delta_t, leftSpeed, rightSpeed: " +str(u_d) + ", " + str(w_d) + ", " + str(delta_t) + ", " + str(leftSpeed) + ", " + str(rightSpeed))
    # odometry
    u, w = get_robot_speeds(leftSpeed, rightSpeed, R, D)
    x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, timestep/1000)

    # Print the current state
    print(f"Estimated position (x, y, phi): ({x:.3f}, {y:.3f}, {phi:.3f})" + f"speed (u, w): ({u:.3f}, {w:.3f})")
    # Update the old encoder values for the next iteration
    oldEncoderValues = encoderValues
    x_old = x
    y_old = y
    phi_old = phi
    prev_timestep = timestep