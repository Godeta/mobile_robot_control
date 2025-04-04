import numpy as np
#unicycle differential drive robot model

def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    """Computes speed of the wheels based on encoder readings
    """
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2*np.pi*(encoderValues[0] - oldEncoderValues[0])/pulses_per_turn
    ang_diff_r = 2*np.pi*(encoderValues[1] - oldEncoderValues[1])/pulses_per_turn

    # Calculate the angular speeds:
    wl = ang_diff_l/delta_t
    wr = ang_diff_r/delta_t

    return wl, wr

def get_robot_speeds(wl, wr, R, D):
    """Computes robot linear and angular speeds"""
    u = R/2.0 * (wr + wl)
    w = R/D * (wr - wl)
    
    return u, w

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi

    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y
    
    return x, y, phi

def wheel_speed_commands(u_d, w_d, D, R):
    """Converts desired speeds to wheel speed commands for a differential-drive robot.
    Inputs:
        u_d = desired linear speed for the robot [m/s]
        w_d = desired angular speed for the robot [rad/s]
        R = radius of the robot wheel [m]
        D = distance between the left and right wheels [m]
    Returns:
        wr_d = desired speed for the right wheel [rad/s]
        wl_d = desired speed for the left wheel [rad/s]
    """
    wr_d = float((2*u_d + D*w_d)/(2*R))
    wl_d = float((2*u_d - D*w_d)/(2*R))

    return wl_d, wr_d

def improved_follow_wall_to_left_obst(kp, kp2, d_fl, d_rl, d_desired, d):
    """ Follows the wall to the left of the robot.
    Input Parameters: 
        kp = controller gain for controlling the distance to the wall;
        kp2 = controller gain for keeping the robot parallel to the wall;
        d_desired = desired robot distance to the wall;
        d_fl = distance to the left wall measured by the front sensor;
        d_rl = distance to the left wall measured by the rear sensor;
        d = measured distance to the obstacle (front sensor);
    Returns:
        u_ref = reference linear speed command;
        w_ref = reference angluar speed command.
    """
    # Variables:
    d_min = 0.05     # [m] minimum admissible distance to the obstacle; 
    d_max = 0.50     # [m] maximum measurable distance by the front sensor;
    u_max = 0.5      # [m/s] linear speed for d = d_max

    u_ref = u_max * d/d_max if d >= d_min else 0
    d_l = (d_fl + d_rl)/2
    w_ref = kp*(d_l - d_desired) + kp2*(d_fl - d_rl)
    
    return u_ref, w_ref

def test_functions():
    #wheel speed
    pulses_per_turn = 72
    delta_t = 0.1  # time step in seconds
    encoderValues = [1506, 1515]  # Accumulated number of pulses for the left [0] and right [1] encoders.
    oldEncoderValues = [1500, 1500]     # Accumulated pulses for the left and right encoders in the previous step

    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)

    print(f'Left wheel speed  = {wl} rad/s.')
    print(f'Right wheel speed = {wr} rad/s.')

    # robot speed, Physical parameters of the robot for the kinematics model
    R = 0.10    # radius of the wheels of the e-puck robot: 20.5mm 
    D = 0.40    # distance between the wheels of the e-puck robot: 52mm

    u, w = get_robot_speeds(wl, wr, R, D)

    print(f"Robot linear speed  = {u} m/s")
    print(f"Robot angular speed = {w} rad/s")

    # robot pose
    x_old, y_old, phi_old = 2.0, 4.0, -np.pi/2  # Robot pose in the previous step
    u = 0.2         # m/s
    w = 0.15        # rad/s
    delta_t = 0.0000001   # s

    x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, delta_t)

    print(f"The new robot pose is: {x:.3f} m, {y:.3f} m, {phi*180/np.pi:.3f} deg.")

    #  wheel speed commands
    print("general model to differential drive robot :")
    # Physical parameters of the robot for the kinematics model
    R = 0.0205    # radius of the wheels of the e-puck robot: 20.5mm 
    D = 0.0520    # distance between the wheels of the e-puck robot: 52mm

    # Desired speeds:
    u_d = 0.1   # [m/s]
    w_d = -0.5  # [rad/s]

    wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)

    print(f"Desired speed of the left wheel  = {wl_d} rad/s")
    print(f"Desired speed of the right wheel = {wr_d} rad/s")

    # wall following / obstacle 
    print("wall following / obstacle :")
    d = 0.5
    d_desired = 0.20    # [m]
    d_fl = 0.18         # [m]
    d_rl = 0.20         # [m]


    # Controller gains: define how the reaction of the robot will be:
    # higher controller gains will result in faster reaction, but can cause oscillations
    kp = 1
    kp2 = 1

    u_ref, w_ref = improved_follow_wall_to_left_obst(kp, kp2, d_fl, d_rl, d_desired, d)
    wl_d, wr_d = wheel_speed_commands(u_ref, w_ref, D, R)

    print(f"Desired linear speed  = {u_ref}m/s")
    print(f"Desired angular speed = {w_ref}rad/s")
    print(f"Desired speed of the left wheel  = {wl_d} rad/s")
    print(f"Desired speed of the right wheel = {wr_d} rad/s")

    return

if __name__ == "__main__":
    test_functions()