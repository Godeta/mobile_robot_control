# test using cline -> very powerfull ! It helped me a lot to improve this demo
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arrow
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation

"""
Lyapunov Controller Simulation
==============================

This simulation demonstrates how a Lyapunov controller works for mobile robot navigation.

cLyapunov Theory Background:
--------------------------
Lyapunov stability theory provides a way to analyze the stability of nonlinear systems.
A Lyapunov function V(x) is a scalar function that satisfies:
1. V(x) > 0 for all x ≠ 0
2. V(0) = 0
3. dV(x)/dt < 0 for all x ≠ 0

For a mobile robot, we can define a Lyapunov function based on the distance to the goal
and the heading error. If we can design control inputs that make this function decrease
over time, the robot will eventually reach the goal.

The Controller:
--------------
The Lyapunov controller computes the control inputs (v, omega) to drive 
the robot towards the goal pose. It uses:
- Distance error: the Euclidean distance between current position and goal
- Heading error: the angle between the robot's heading and the direction to the goal

The gains k_l and k_psi control the robot's linear and angular velocity, respectively.
The control inputs are designed to ensure that the Lyapunov function decreases over time,
guaranteeing that the robot will reach the goal while maintaining stability.
"""

class LyapunovController:
    def __init__(self, k_l=1.0, k_psi=2.0):
        """
        Initialize the Lyapunov controller with gain parameters.
        
        Parameters:
        -----------
        k_l : float
            Gain for linear velocity control. Higher values make the robot move faster.
        k_psi : float
            Gain for angular velocity control. Higher values make the robot turn faster.
        """
        self.k_l = k_l
        self.k_psi = k_psi
    
    def compute_lyapunov_value(self, current_pose, goal_pose):
        """
        Compute the value of the Lyapunov function, which represents the 'energy' 
        or 'distance' from the desired state.
        
        A decreasing Lyapunov value indicates the system is approaching the goal.
        """
        x, y, theta = current_pose
        x_goal, y_goal = goal_pose
        
        # Compute errors
        dx = x_goal - x
        dy = y_goal - y
        
        # Distance and heading error
        distance = np.sqrt(dx**2 + dy**2)
        heading_error = np.arctan2(dy, dx) - theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Lyapunov function: V = distance² + heading_error²
        # This is positive definite and zero only at the goal
        lyapunov_value = distance**2 + heading_error**2
        
        return lyapunov_value, distance, heading_error

    def compute_velocities(self, current_pose, goal_pose):
        """
        Compute control velocities (v, omega) using Lyapunov control law.
        
        Parameters:
        -----------
        current_pose : list or array [x, y, theta]
            Current robot position and orientation
        goal_pose : list or array [x_goal, y_goal]
            Target position
            
        Returns:
        --------
        v : float
            Linear velocity command
        omega : float
            Angular velocity command
        """
        # Current and goal positions
        x, y, theta = current_pose
        x_goal, y_goal = goal_pose

        # Compute errors
        dx = x_goal - x
        dy = y_goal - y

        # Distance and heading error
        distance = np.sqrt(dx**2 + dy**2)
        heading_error = np.arctan2(dy, dx) - theta

        # Normalize heading error to [-π, π]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Step 1: Compute linear velocity
        # We want to move faster when we're far from the goal and slow down as we approach
        # The cos(heading_error) term ensures we slow down if not facing the goal
        v = self.k_l * distance * np.cos(heading_error)
        
        # Step 2: Compute angular velocity
        # First term: Makes the robot turn toward the goal
        # Second term: Additional correction based on heading error
        omega = (self.k_l * np.sin(heading_error) * np.cos(heading_error) + 
                 self.k_psi * np.tanh(heading_error))

        return v, omega, distance, heading_error

def draw_robot(ax, x, y, theta, radius=0.3, color='blue', alpha=0.7):
    """Draw a simple representation of the robot as a circle with a direction indicator"""
    robot = Circle((x, y), radius, fill=True, color=color, alpha=alpha)
    ax.add_patch(robot)
    
    # Add direction indicator (heading)
    head_x = x + radius * np.cos(theta)
    head_y = y + radius * np.sin(theta)
    heading = Arrow(x, y, radius * np.cos(theta), radius * np.sin(theta), 
                    width=0.2, color='black')
    ax.add_patch(heading)
    
    return robot, heading

def simulate_robot_tracking_step_by_step(start_pose, goal_pose, dt=0.1, max_time=20, 
                                        k_l=1.0, k_psi=2.0, save_animation=False):
    """
    Simulate and visualize the robot's motion using the Lyapunov controller step by step.
    
    Parameters:
    -----------
    start_pose : list [x, y, theta]
        Initial robot position and orientation
    goal_pose : list [x_goal, y_goal]
        Target position
    dt : float
        Time step for simulation
    max_time : float
        Maximum simulation time
    k_l, k_psi : float
        Controller gain parameters
    save_animation : bool
        Whether to save the animation as a GIF file
    """
    controller = LyapunovController(k_l, k_psi)
    
    # Initialize arrays for tracking
    time_points = [0]
    x_history = [start_pose[0]]
    y_history = [start_pose[1]]
    theta_history = [start_pose[2]]
    v_history = [0]
    omega_history = [0]
    distance_history = []
    heading_error_history = []
    lyapunov_history = []
    
    current_pose = list(start_pose)
    
    # Calculate initial Lyapunov value and errors
    lyap_val, dist, head_err = controller.compute_lyapunov_value(current_pose, goal_pose)
    lyapunov_history.append(lyap_val)
    distance_history.append(dist)
    heading_error_history.append(head_err)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # Main trajectory plot
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title('Robot Trajectory and Control Vectors')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.grid(True)
    
    # Plot goal and start positions
    ax1.plot(goal_pose[0], goal_pose[1], 'ro', markersize=10, label='Goal')
    ax1.plot(start_pose[0], start_pose[1], 'go', markersize=10, label='Start')
    
    # Initialize path line
    path_line, = ax1.plot(x_history, y_history, 'b-', label='Robot Path')
    
    # Draw initial robot
    robot, heading = draw_robot(ax1, start_pose[0], start_pose[1], start_pose[2])
    
    # Control inputs plot
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title('Control Inputs Over Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Value')
    ax2.grid(True)
    
    v_line, = ax2.plot(time_points, v_history, 'g-', label='Linear Velocity (v)')
    omega_line, = ax2.plot(time_points, omega_history, 'r-', label='Angular Velocity (ω)')
    ax2.legend()
    
    # Lyapunov function plot
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title('Lyapunov Function Value')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('V(x)')
    ax3.grid(True)
    
    lyap_line, = ax3.plot(time_points, lyapunov_history, 'b-')
    
    # Error metrics plot
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title('Error Metrics')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error')
    ax4.grid(True)
    
    dist_line, = ax4.plot(time_points, distance_history, 'g-', label='Distance Error')
    head_line, = ax4.plot(time_points, heading_error_history, 'r-', label='Heading Error')
    ax4.legend()
    
    # Text annotations for explanation
    explanation_text = ax1.text(0.05, 0.95, '', transform=ax1.transAxes, 
                               verticalalignment='top', bbox=dict(boxstyle='round', 
                               facecolor='wheat', alpha=0.5))
    
    # Phase indicator
    phase_text = fig.text(0.5, 0.95, 'Initialization Phase', 
                         horizontalalignment='center', fontsize=14,
                         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    # Animation function
    def update(frame):
        nonlocal current_pose, robot, heading
        t = frame * dt
        
        # Compute control inputs using the Lyapunov controller
        v, omega, distance, heading_error = controller.compute_velocities(current_pose, goal_pose)
        
        # Update robot pose using the control inputs and the robot's dynamics
        current_pose[0] += v * np.cos(current_pose[2]) * dt
        current_pose[1] += v * np.sin(current_pose[2]) * dt
        current_pose[2] += omega * dt
        
        # Normalize theta to [-π, π]
        current_pose[2] = np.arctan2(np.sin(current_pose[2]), np.cos(current_pose[2]))
        
        # Calculate Lyapunov value
        lyap_val, _, _ = controller.compute_lyapunov_value(current_pose, goal_pose)
        
        # Store history
        time_points.append(t)
        x_history.append(current_pose[0])
        y_history.append(current_pose[1])
        theta_history.append(current_pose[2])
        v_history.append(v)
        omega_history.append(omega)
        lyapunov_history.append(lyap_val)
        distance_history.append(distance)
        heading_error_history.append(heading_error)
        
        # Update plots
        path_line.set_data(x_history, y_history)
        
        # Remove old robot and heading
        robot.remove()
        heading.remove()
        
        # Draw new robot position
        robot, heading = draw_robot(ax1, current_pose[0], current_pose[1], current_pose[2])
        
        # Update control inputs plot
        v_line.set_data(time_points, v_history)
        omega_line.set_data(time_points, omega_history)
        ax2.relim()
        ax2.autoscale_view()
        
        # Update Lyapunov plot
        lyap_line.set_data(time_points, lyapunov_history)
        ax3.relim()
        ax3.autoscale_view()
        
        # Update error metrics plot
        dist_line.set_data(time_points, distance_history)
        head_line.set_data(time_points, heading_error_history)
        ax4.relim()
        ax4.autoscale_view()
        
        # Update trajectory plot limits
        ax1.relim()
        ax1.autoscale_view()
        
        # Update explanation text based on the phase of the trajectory
        if t < 2:
            phase_text.set_text('Initial Approach Phase')
            explanation = (
                "Initial Phase:\n"
                f"- Distance to goal: {distance:.2f}\n"
                f"- Heading error: {heading_error:.2f} rad\n"
                f"- Linear velocity (v): {v:.2f}\n"
                f"- Angular velocity (ω): {omega:.2f}\n\n"
                "The controller is calculating velocities\n"
                "to minimize the Lyapunov function."
            )
        elif distance < 1.0 and abs(heading_error) > 0.1:
            phase_text.set_text('Alignment Phase')
            explanation = (
                "Alignment Phase:\n"
                f"- Distance to goal: {distance:.2f}\n"
                f"- Heading error: {heading_error:.2f} rad\n"
                f"- Linear velocity (v): {v:.2f}\n"
                f"- Angular velocity (ω): {omega:.2f}\n\n"
                "The robot is close to the goal and\n"
                "adjusting its heading to align properly."
            )
        elif distance < 0.5:
            phase_text.set_text('Final Approach Phase')
            explanation = (
                "Final Approach Phase:\n"
                f"- Distance to goal: {distance:.2f}\n"
                f"- Heading error: {heading_error:.2f} rad\n"
                f"- Linear velocity (v): {v:.2f}\n"
                f"- Angular velocity (ω): {omega:.2f}\n\n"
                "The robot is very close to the goal\n"
                "and making final adjustments."
            )
        else:
            phase_text.set_text('Tracking Phase')
            explanation = (
                "Tracking Phase:\n"
                f"- Distance to goal: {distance:.2f}\n"
                f"- Heading error: {heading_error:.2f} rad\n"
                f"- Linear velocity (v): {v:.2f}\n"
                f"- Angular velocity (ω): {omega:.2f}\n\n"
                "The robot is moving toward the goal\n"
                "with decreasing Lyapunov function value."
            )
        
        explanation_text.set_text(explanation)
        
        # Check if goal is reached
        if distance < 0.1:
            phase_text.set_text('Goal Reached!')
            explanation = (
                "Goal Reached!\n"
                f"- Final distance: {distance:.2f}\n"
                f"- Final heading error: {heading_error:.2f} rad\n\n"
                "The Lyapunov controller has successfully\n"
                "guided the robot to the goal position."
            )
            explanation_text.set_text(explanation)
            return robot, heading, path_line, v_line, omega_line, lyap_line, dist_line, head_line, explanation_text, phase_text
        
        return robot, heading, path_line, v_line, omega_line, lyap_line, dist_line, head_line, explanation_text, phase_text
    
    # Create animation
    frames = int(max_time / dt)
    try:
        # Try with blitting first (faster)
        ani = animation.FuncAnimation(fig, update, frames=frames, interval=50, blit=True)
    except Exception as e:
        print(f"Warning: Blitting animation failed with error: {e}")
        print("Falling back to non-blitting animation (slower but more compatible)")
        
        # Define a simpler update function without blitting
        def simple_update(frame):
            nonlocal current_pose, robot, heading
            t = frame * dt
            
            # Compute control inputs using the Lyapunov controller
            v, omega, distance, heading_error = controller.compute_velocities(current_pose, goal_pose)
            
            # Update robot pose using the control inputs and the robot's dynamics
            current_pose[0] += v * np.cos(current_pose[2]) * dt
            current_pose[1] += v * np.sin(current_pose[2]) * dt
            current_pose[2] += omega * dt
            
            # Normalize theta to [-π, π]
            current_pose[2] = np.arctan2(np.sin(current_pose[2]), np.cos(current_pose[2]))
            
            # Calculate Lyapunov value
            lyap_val, _, _ = controller.compute_lyapunov_value(current_pose, goal_pose)
            
            # Store history
            time_points.append(t)
            x_history.append(current_pose[0])
            y_history.append(current_pose[1])
            theta_history.append(current_pose[2])
            v_history.append(v)
            omega_history.append(omega)
            lyapunov_history.append(lyap_val)
            distance_history.append(distance)
            heading_error_history.append(heading_error)
            
            # Update plots
            path_line.set_data(x_history, y_history)
            
            # Remove old robot and heading
            robot.remove()
            heading.remove()
            
            # Draw new robot position
            robot, heading = draw_robot(ax1, current_pose[0], current_pose[1], current_pose[2])
            
            # Update control inputs plot
            v_line.set_data(time_points, v_history)
            omega_line.set_data(time_points, omega_history)
            ax2.relim()
            ax2.autoscale_view()
            
            # Update Lyapunov plot
            lyap_line.set_data(time_points, lyapunov_history)
            ax3.relim()
            ax3.autoscale_view()
            
            # Update error metrics plot
            dist_line.set_data(time_points, distance_history)
            head_line.set_data(time_points, heading_error_history)
            ax4.relim()
            ax4.autoscale_view()
            
            # Update trajectory plot limits
            ax1.relim()
            ax1.autoscale_view()
            
            # Update explanation text based on the phase of the trajectory
            if t < 2:
                phase_text.set_text('Initial Approach Phase')
                explanation = (
                    "Initial Phase:\n"
                    f"- Distance to goal: {distance:.2f}\n"
                    f"- Heading error: {heading_error:.2f} rad\n"
                    f"- Linear velocity (v): {v:.2f}\n"
                    f"- Angular velocity (ω): {omega:.2f}\n\n"
                    "The controller is calculating velocities\n"
                    "to minimize the Lyapunov function."
                )
            elif distance < 1.0 and abs(heading_error) > 0.1:
                phase_text.set_text('Alignment Phase')
                explanation = (
                    "Alignment Phase:\n"
                    f"- Distance to goal: {distance:.2f}\n"
                    f"- Heading error: {heading_error:.2f} rad\n"
                    f"- Linear velocity (v): {v:.2f}\n"
                    f"- Angular velocity (ω): {omega:.2f}\n\n"
                    "The robot is close to the goal and\n"
                    "adjusting its heading to align properly."
                )
            elif distance < 0.5:
                phase_text.set_text('Final Approach Phase')
                explanation = (
                    "Final Approach Phase:\n"
                    f"- Distance to goal: {distance:.2f}\n"
                    f"- Heading error: {heading_error:.2f} rad\n"
                    f"- Linear velocity (v): {v:.2f}\n"
                    f"- Angular velocity (ω): {omega:.2f}\n\n"
                    "The robot is very close to the goal\n"
                    "and making final adjustments."
                )
            else:
                phase_text.set_text('Tracking Phase')
                explanation = (
                    "Tracking Phase:\n"
                    f"- Distance to goal: {distance:.2f}\n"
                    f"- Heading error: {heading_error:.2f} rad\n"
                    f"- Linear velocity (v): {v:.2f}\n"
                    f"- Angular velocity (ω): {omega:.2f}\n\n"
                    "The robot is moving toward the goal\n"
                    "with decreasing Lyapunov function value."
                )
            
            explanation_text.set_text(explanation)
            
            # Check if goal is reached
            if distance < 0.1:
                phase_text.set_text('Goal Reached!')
                explanation = (
                    "Goal Reached!\n"
                    f"- Final distance: {distance:.2f}\n"
                    f"- Final heading error: {heading_error:.2f} rad\n\n"
                    "The Lyapunov controller has successfully\n"
                    "guided the robot to the goal position."
                )
                explanation_text.set_text(explanation)
        
        # Create non-blitting animation
        ani = animation.FuncAnimation(fig, simple_update, frames=frames, interval=50, blit=False)
    
    # Save animation if requested
    if save_animation:
        try:
            ani.save('lyapunov_controller_simulation.gif', writer='pillow', fps=20)
        except Exception as e:
            print(f"Warning: Could not save animation: {e}")
    
    plt.show()
    
    return x_history, y_history, theta_history, v_history, omega_history, lyapunov_history

def compare_gain_parameters(save_figure=False):
    """
    Compare the effect of different gain parameters on the Lyapunov controller performance.
    """
    # Create a figure with subplots
    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    axs = axs.flatten()
    
    # Different gain combinations to test
    gains = [
        (0.5, 1.0, "Low k_l, Medium k_psi"),
        (1.0, 2.0, "Medium k_l, High k_psi"),
        (2.0, 1.0, "High k_l, Medium k_psi"),
        (0.5, 3.0, "Low k_l, Very High k_psi")
    ]
    
    start_pose = [0, 0, 0]  # x, y, theta
    goal_pose = [5, 5]      # x, y
    
    for i, (k_l, k_psi, title) in enumerate(gains):
        controller = LyapunovController(k_l, k_psi)
        
        # Initialize arrays for tracking
        x_history = [start_pose[0]]
        y_history = [start_pose[1]]
        theta_history = [start_pose[2]]
        
        current_pose = list(start_pose)
        dt = 0.1
        max_time = 20
        
        # Simulate
        for _ in np.arange(0, max_time, dt):
            # Compute control inputs
            v, omega, _, _ = controller.compute_velocities(current_pose, goal_pose)
            
            # Update robot pose
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
        
        # Plot
        axs[i].plot(x_history, y_history, 'b-', label='Robot Path')
        axs[i].plot(start_pose[0], start_pose[1], 'go', label='Start')
        axs[i].plot(goal_pose[0], goal_pose[1], 'ro', label='Goal')
        axs[i].set_title(f'{title} (k_l={k_l}, k_psi={k_psi})')
        axs[i].set_xlabel('X Position')
        axs[i].set_ylabel('Y Position')
        axs[i].legend()
        axs[i].grid(True)
    
    plt.tight_layout()
    
    # Save figure if requested
    if save_figure:
        try:
            plt.savefig('videos/lyapunov_gain_comparison.png', dpi=300, bbox_inches='tight')
            print("Gain comparison figure saved as 'lyapunov_gain_comparison.png'")
        except Exception as e:
            print(f"Warning: Could not save figure: {e}")
    
    plt.show()

# Main simulation function with interactive explanation
def run_static_simulation():
    """
    Run a static (non-animated) simulation of the Lyapunov controller.
    This is useful for systems where animation might not work well.
    """
    print("Running static simulation of Lyapunov controller...")
    
    # Parameters
    start_pose = [0, 0, 0]  # x, y, theta
    goal_pose = [5, 5]      # x, y
    dt = 0.1
    max_time = 20
    k_l = 1.0
    k_psi = 2.0
    
    controller = LyapunovController(k_l, k_psi)
    
    # Initialize arrays for tracking
    time_points = [0]
    x_history = [start_pose[0]]
    y_history = [start_pose[1]]
    theta_history = [start_pose[2]]
    v_history = [0]
    omega_history = [0]
    distance_history = []
    heading_error_history = []
    lyapunov_history = []
    
    current_pose = list(start_pose)
    
    # Calculate initial Lyapunov value and errors
    lyap_val, dist, head_err = controller.compute_lyapunov_value(current_pose, goal_pose)
    lyapunov_history.append(lyap_val)
    distance_history.append(dist)
    heading_error_history.append(head_err)
    
    # Simulate
    for t in np.arange(0, max_time, dt):
        # Compute control inputs
        v, omega, distance, heading_error = controller.compute_velocities(current_pose, goal_pose)
        
        # Update robot pose
        current_pose[0] += v * np.cos(current_pose[2]) * dt
        current_pose[1] += v * np.sin(current_pose[2]) * dt
        current_pose[2] += omega * dt
        
        # Normalize theta
        current_pose[2] = np.arctan2(np.sin(current_pose[2]), np.cos(current_pose[2]))
        
        # Calculate Lyapunov value
        lyap_val, _, _ = controller.compute_lyapunov_value(current_pose, goal_pose)
        
        # Store history
        time_points.append(t)
        x_history.append(current_pose[0])
        y_history.append(current_pose[1])
        theta_history.append(current_pose[2])
        v_history.append(v)
        omega_history.append(omega)
        lyapunov_history.append(lyap_val)
        distance_history.append(distance)
        heading_error_history.append(heading_error)
        
        # Check if goal is reached
        if distance < 0.1:
            print(f"Goal reached at time t={t:.2f}s")
            break
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # Main trajectory plot
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title('Robot Trajectory')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.grid(True)
    
    # Plot trajectory
    ax1.plot(x_history, y_history, 'b-', label='Robot Path')
    ax1.plot(start_pose[0], start_pose[1], 'go', markersize=10, label='Start')
    ax1.plot(goal_pose[0], goal_pose[1], 'ro', markersize=10, label='Goal')
    
    # Draw robot at key points
    num_points = min(10, len(x_history))
    indices = np.linspace(0, len(x_history)-1, num_points, dtype=int)
    
    for i in indices:
        draw_robot(ax1, x_history[i], y_history[i], theta_history[i], 
                  radius=0.2, alpha=0.3 + 0.7*i/len(x_history))
    
    ax1.legend()
    
    # Control inputs plot
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title('Control Inputs Over Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Value')
    ax2.grid(True)
    
    ax2.plot(time_points, v_history, 'g-', label='Linear Velocity (v)')
    ax2.plot(time_points, omega_history, 'r-', label='Angular Velocity (ω)')
    ax2.legend()
    
    # Lyapunov function plot
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title('Lyapunov Function Value')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('V(x)')
    ax3.grid(True)
    
    ax3.plot(time_points, lyapunov_history, 'b-')
    
    # Error metrics plot
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title('Error Metrics')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error')
    ax4.grid(True)
    
    ax4.plot(time_points, distance_history, 'g-', label='Distance Error')
    ax4.plot(time_points, heading_error_history, 'r-', label='Heading Error')
    ax4.legend()
    
    # Add explanation text
    explanation = (
        "Lyapunov Controller Explanation:\n\n"
        "1. The controller uses distance and heading errors\n"
        "   to compute control inputs (v, ω).\n\n"
        "2. The Lyapunov function V(x) = distance² + heading_error²\n"
        "   decreases over time, ensuring stability.\n\n"
        "3. Linear velocity (v) is proportional to distance\n"
        "   and scaled by cos(heading_error).\n\n"
        "4. Angular velocity (ω) has two components:\n"
        "   - A term to align with the goal\n"
        "   - A term to correct heading errors"
    )
    
    fig.text(0.5, 0.95, 'Lyapunov Controller Step-by-Step Explanation', 
             horizontalalignment='center', fontsize=14,
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    fig.text(0.02, 0.5, explanation, fontsize=10, 
             verticalalignment='center',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout(rect=[0.15, 0, 1, 0.95])
    
    # Save figure
    try:
        plt.savefig('videos/lyapunov_static_simulation.png', dpi=300, bbox_inches='tight')
        print("Static simulation figure saved as 'lyapunov_static_simulation.png'")
    except Exception as e:
        print(f"Warning: Could not save figure: {e}")
    
    plt.show()

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

def pendulum_lyapunov_demo():
    """Demonstrate Lyapunov stability analysis for a pendulum without solving ODE"""
    # Pendulum parameters
    g = 9.81
    l = 1.0
    m = 1.0
    k = 0.0  # Start with no friction

    # Initial conditions
    theta0 = np.pi/4
    theta_dot0 = 0.0

    # Simulation parameters
    dt = 0.05
    t_max = 10.0
    time = np.arange(0, t_max, dt)

    # Create figure
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle("Pendulum Stability Analysis using Lyapunov Theory", y=0.98)

    # Create subplots
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)

    # Set initial plot limits
    for ax in [ax1, ax2, ax3]:
        ax.set_xlim(0, t_max)
    ax4.set_xlim(-np.pi, np.pi)
    ax4.set_ylim(-5, 5)

    # Initialize data arrays
    theta = [theta0]
    theta_dot = [theta_dot0]
    V = [0.5*m*l**2*theta_dot0**2 + m*g*l*(1 - np.cos(theta0))]
    V_dot = [0.0]

    # Create plot objects
    line_theta, = ax1.plot([], [], 'b-', lw=2)
    line_V, = ax2.plot([], [], 'r-', lw=2)
    line_V_dot, = ax3.plot([], [], 'g-', lw=2)
    phase_plot, = ax4.plot([], [], 'k.', markersize=4)

    # Format plots
    ax1.set_title('Pendulum Angle')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('θ (rad)')
    ax1.grid(True)

    ax2.set_title('Lyapunov Function (Energy)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('V(θ, ω)')
    ax2.grid(True)

    ax3.set_title('Lyapunov Derivative')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('dV/dt')
    ax3.grid(True)

    ax4.set_title('Phase Portrait')
    ax4.set_xlabel('θ (rad)')
    ax4.set_ylabel('ω (rad/s)')
    ax4.grid(True)

    # Add explanation text
    explanation_text = fig.text(0.01, 0.90,
        "Lyapunov Stability Analysis:\n"
        "1. V(θ,ω) = ½ml²ω² + mgl(1 - cosθ) (Total Energy)\n"
        "2. V(0,0) = 0 and V > 0 elsewhere → Positive Definite\n"
        "3. dV/dt = -kω² (≤ 0 when friction exists)\n"
        "→ System is stable (asymptotically stable with friction)",
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Add controls
    axcolor = 'lightgoldenrodyellow'
    ax_theta = plt.axes([0.15, 0.05, 0.3, 0.03], facecolor=axcolor)
    ax_omega = plt.axes([0.15, 0.01, 0.3, 0.03], facecolor=axcolor)

    slider_theta = Slider(ax_theta, 'Initial θ (rad)', -np.pi, np.pi, valinit=theta0)
    slider_omega = Slider(ax_omega, 'Initial ω (rad/s)', -5.0, 5.0, valinit=theta_dot0)

    # Friction checkbox
    checkbox_ax = plt.axes([0.6, 0.02, 0.2, 0.04])
    friction_check = Button(checkbox_ax, 'Toggle Friction (k)', color=axcolor)

    def update(val):
        """Update simulation with new parameters"""
        nonlocal theta, theta_dot, V, V_dot, k
        theta = [slider_theta.val]
        theta_dot = [slider_omega.val]
        V = [0.5*m*l**2*theta_dot[0]**2 + m*g*l*(1 - np.cos(theta[0]))]
        V_dot = [0.0]

        # Run simulation
        for t in time[1:]:
            # Calculate derivatives
            theta_dot_new = theta_dot[-1] + (-g/l*np.sin(theta[-1]) - k/m*theta_dot[-1])*dt
            theta_new = theta[-1] + theta_dot[-1]*dt

            # Store values
            theta.append(theta_new)
            theta_dot.append(theta_dot_new)

            # Calculate Lyapunov function and derivative
            V_new = 0.5*m*l**2*theta_dot_new**2 + m*g*l*(1 - np.cos(theta_new))
            V.append(V_new)
            V_dot.append((V_new - V[-2])/dt)  # Approximate derivative

        # Update plots
        line_theta.set_data(time[:len(theta)], theta)
        line_V.set_data(time[:len(V)], V)
        line_V_dot.set_data(time[:len(V_dot)], V_dot)
        phase_plot.set_data(theta, theta_dot)

        # Update plot limits
        for ax in [ax1, ax2, ax3]:
            ax.relim()
            ax.autoscale_view()

        fig.canvas.draw_idle()

    def toggle_friction(event):
        """Toggle friction coefficient"""
        nonlocal k
        k = 2.0 if k == 0.0 else 0.0
        update(None)

    # Connect controls
    slider_theta.on_changed(update)
    slider_omega.on_changed(update)
    friction_check.on_clicked(toggle_friction)

    # Initial update
    update(None)

    plt.show()



def main():
    """
    Run the main Lyapunov controller simulation with step-by-step explanation.
    """
    print("Lyapunov Controller Simulation")
    print("==============================")
    print("\nThis simulation demonstrates how a Lyapunov controller guides a mobile robot to a goal position.")
    print("\nThe controller uses two gain parameters:")
    print("- k_l: Controls the linear velocity (higher values = faster linear motion)")
    print("- k_psi: Controls the angular velocity (higher values = faster turning)")
    print("\nThe simulation will show:")
    print("1. The robot's trajectory")
    print("2. Control inputs (v, omega) over time")
    print("3. Lyapunov function value (should decrease over time)")
    print("4. Error metrics (distance and heading errors)")
    print("\nStarting simulation...\n")
    
    # Default parameters
    start_pose = [0, 0, 0]  # x, y, theta
    goal_pose = [5, 5]      # x, y
    
    # Run the static simulation which is more reliable
    run_static_simulation()
    
    # Compare different gain parameters
    print("\nNow comparing different gain parameters to show their effect on the controller...")
    compare_gain_parameters(save_figure=True)

    # lyapunov pendulum
    pendulum_lyapunov_demo()

if __name__ == "__main__":
    main()