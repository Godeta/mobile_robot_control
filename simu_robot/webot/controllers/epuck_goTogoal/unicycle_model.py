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

def get_pose_error(xd, yd, x, y, phi):
    """ Returns the position and orientation errors. 
        Orientation error is bounded between -pi and +pi radians.
    """
    # Position error:
    x_err = xd - x
    y_err = yd - y
    dist_err = np.sqrt(x_err**2 + y_err**2)

    # Orientation error
    phi_d = np.arctan2(y_err,x_err)
    phi_err = phi_d - phi

    # Limits the error to (-pi, pi):
    phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))

    return dist_err, phi_err_correct

def pid_controller(e, e_prev, e_acc, delta_t, kp=1.0, kd=0, ki=0):
    """ PID algortithm: must be executed every delta_t seconds
    The error e must be calculated as: e = desired_value - actual_value
    e_prev contains the error calculated in the previous step.
    e_acc contains the integration (accumulation) term.
    """
    P = kp*e                      # Proportional term; kp is the proportional gain
    I = e_acc + ki*e*delta_t    # Intergral term; ki is the integral gain
    D = kd*(e - e_prev)/delta_t   # Derivative term; kd is the derivative gain

    output = P + I + D              # controller output

    # store values for the next iteration
    e_prev = e     # error value in the previous interation (to calculate the derivative term)
    e_acc = I      # accumulated error value (to calculate the integral term)

    return output, e_prev, e_acc

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

    # pose error 
    # Actual robot pose:
    x, y, phi = 0, 0, np.pi/4

    # Desired robot position:
    xd, yd = -1, 1

    position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)

    print(f'Distance error    = {position_err} m.')
    print(f'Orientation error = {orientation_err} rad.')

    # pid function
    # The values below are initialized to test the function. 
    # When implementing this, you must update e_prev and e_acc properly at every step.
    e = orientation_err
    e_prev = orientation_err*0.9
    e_acc = 0

    delta_t = 0.01

    # Controller gains:
    kp = 0.5
    kd = 0.01
    ki = 0.1

    # Obtain the desired angular speed:
    w_d, e_prev, e_acc = pid_controller(e, e_prev, e_acc, delta_t, kp, kd, ki)

    print(f'Desired angular speed w_d = {w_d} rad/s.')
    print(f'Previous error = {e_prev} rad.')
    print(f'Accumulated error = {e_acc}.')

    return

if __name__ == "__main__":
    test_functions()