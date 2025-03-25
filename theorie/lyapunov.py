import numpy as np
import matplotlib.pyplot as plt

# the Lyapunov controller, which computes the control inputs (v, omega) to drive 
# the robot towards the goal pose. The simulate_robot_tracking function simulates
# the robot's motion using the Lyapunov controller and plots the robot's trajectory.
# The Lyapunov controller uses the distance and heading error between the current pose and 
# the goal pose to compute the control inputs. The gains k_l and k_psi control the robot's speed 
# and angular velocity, respectively. The control inputs are designed to drive the system towards the goal pose, 
# ensuring that the robot reaches the desired position while maintaining stability.
# The simulation results show the robot's trajectory as it approaches the goal pose, 
# demonstrating the effectiveness of the Lyapunov-based trajectory tracking algorithm.

class LyapunovController:
    def __init__(self, k_l=1.0, k_psi=2.0):
        # Gain parameters for the Lyapunov controller
        self.k_l = k_l
        self.k_psi = k_psi

    def compute_velocities(self, current_pose, goal_pose):
        # Current and goal positions
        x, y, theta = current_pose
        x_goal, y_goal = goal_pose

        # Compute errors
        dx = x_goal - x
        dy = y_goal - y

        # Distance and heading error
        distance = np.sqrt(dx**2 + dy**2)
        heading_error = np.arctan2(dy, dx) - theta

        # Normalize heading error
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Compute velocities using the Lyapunov controller
        # The control inputs (v, omega) are designed to drive the system towards the goal pose
        v = self.k_l * distance * np.cos(heading_error)
        omega = (self.k_l * np.sin(heading_error) * np.cos(heading_error) + 
                 self.k_psi * np.tanh(heading_error))

        return v, omega

def simulate_robot_tracking(start_pose, goal_pose, dt=0.1, max_time=20):
    controller = LyapunovController()

    # Initialize arrays for tracking
    x_history = [start_pose[0]]
    y_history = [start_pose[1]]
    theta_history = [start_pose[2]]

    current_pose = list(start_pose)

    for _ in np.arange(0, max_time, dt):
        # Compute control inputs using the Lyapunov controller
        v, omega = controller.compute_velocities(current_pose, goal_pose)

        # Update robot pose using the control inputs and the robot's dynamics
        current_pose[0] += v * np.cos(current_pose[2]) * dt
        current_pose[1] += v * np.sin(current_pose[2]) * dt
        current_pose[2] += omega * dt

        # Store history
        x_history.append(current_pose[0])
        y_history.append(current_pose[1])
        theta_history.append(current_pose[2])

        # Check if goal is reached
        if np.sqrt((current_pose[0]-goal_pose[0])**2 + 
                   (current_pose[1]-goal_pose[1])**2) < 0.1:
            break

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(x_history, y_history, 'b-', label='Robot Path')
    plt.plot(start_pose[0], start_pose[1], 'go', label='Start')
    plt.plot(goal_pose[0], goal_pose[1], 'ro', label='Goal')
    plt.title('Lyapunov-Based Trajectory Tracking')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)
    plt.show()

# Simulation
start_pose = [0, 0, 0]  # x, y, theta
goal_pose = [5, 5]      # x, y

simulate_robot_tracking(start_pose, goal_pose)


# demo differential wheels :
# In[1]:


import numpy as np


# In[2]:


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


# Test the function with different encoder values and delta_t.

# In[3]:


pulses_per_turn = 72
delta_t = 0.1  # time step in seconds
encoderValues = [1506, 1515]  # Accumulated number of pulses for the left [0] and right [1] encoders.
oldEncoderValues = [1500, 1500]     # Accumulated pulses for the left and right encoders in the previous step

wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)

print(f'Left wheel speed  = {wl} rad/s.')
print(f'Right wheel speed = {wr} rad/s.')

# In[4]:


def get_robot_speeds(wl, wr, R, D):
    """Computes robot linear and angular speeds"""
    u = R/2.0 * (wr + wl)
    w = R/D * (wr - wl)
    
    return u, w

# Physical parameters of the robot for the kinematics model
R = 0.10    # radius of the wheels of the e-puck robot: 20.5mm 
D = 0.40    # distance between the wheels of the e-puck robot: 52mm

u, w = get_robot_speeds(wl, wr, R, D)

print(f"Robot linear speed  = {u} m/s")
print(f"Robot angular speed = {w} rad/s")

# Finally, the new robot pose can be calculated based on the kinematics model, robot speed and previous pose.

# In[5]:


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


# Test the function with different speeds and poses.
x_old, y_old, phi_old = 2.0, 4.0, -np.pi/2  # Robot pose in the previous step
u = 0.2         # m/s
w = 0.15        # rad/s
delta_t = 0.1   # s

x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, delta_t)

print(f"The new robot pose is: {x:.3f} m, {y:.3f} m, {phi*180/np.pi:.3f} deg.")