# this program illustrates a simulation of the e-puck robot moving to a poing and following a trajectory

import tkinter as tk
import math
import time
import random

class EPuck2:
    """
    A class to model the e-Puck 2 robot's kinematics, 
    convert Cartesian to Polar coordinates, and implement different control algorithms.
    """
    def __init__(self, canvas, x=50, y=50, theta=0):
        """
        Initialize the robot with its position and orientation.
        """
        self.canvas = canvas
        self.x = x
        self.y = y
        self.theta = theta
        
        # Robot Constants
        self.wheel_radius = 20.5  # mm
        self.axle_length = 52  # mm
        self.max_speed = 0.25  # m/s
        self.max_rotation = 6.28  # rad/s
        
        # Tkinter scaling factor (1m = 100 pixels for visualization)
        self.scale = 100 
        self.running = False  # Flag to stop movement
        
        # Trajectory
        self.trajectory = []
        self.goal_point = None
    
    def cartesian_to_polar(self, goal_x, goal_y):
        """
        Convert goal coordinates from Cartesian to Polar relative to the robot.
        """
        dx = goal_x - self.x
        dy = goal_y - self.y
        
        rho = math.sqrt(dx**2 + dy**2)  # Distance to the goal
        gamma = math.atan2(dy, dx)  # Angle to the goal
        delta = gamma - self.theta  # Angle difference
        
        # Normalize delta between -pi and pi
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        
        return rho, gamma, delta
    
    def move_to_point(self, goal_x, goal_y, Kp=2.0, dt=0.1):
        """
        Move the robot to the goal position using a proportional controller.
        """
        self.running = True
        rho, gamma, delta = self.cartesian_to_polar(goal_x, goal_y)
        
        while rho > 5 and self.running:  # Stop when close enough or when stopped manually
            # Compute control signals
            linear_velocity = min(Kp * rho / self.scale, self.max_speed)
            angular_velocity = min(Kp * delta, self.max_rotation)
            
            # Update position and orientation
            self.theta += angular_velocity * dt
            self.x += linear_velocity * math.cos(self.theta) * self.scale * dt
            self.y += linear_velocity * math.sin(self.theta) * self.scale * dt
            
            # Recalculate distance and angle to goal
            rho, gamma, delta = self.cartesian_to_polar(goal_x, goal_y)
            
            # Update visualization
            self.canvas.delete("robot")
            self.draw()
            root.update()
            time.sleep(dt)

    def stop_movement(self):
        """
        Stop the robot's movement by setting running to False.
        """
        self.running = False
    
    def follow_trajectory(self, Kp=1.5, Ki=0.01, Kd=0.1, dt=0.1):
        """
        Follow the defined trajectory using a PID controller.
        """
        if not self.trajectory:
            return  # No trajectory to follow
        self.running = True
        integral = 0
        previous_error = 0
        if self.running == True:
            for goal_x, goal_y in self.trajectory:
                while True:
                    rho, gamma, delta = self.cartesian_to_polar(goal_x, goal_y)
                    
                    if rho < 5:  # Stop when close enough
                        break
                    
                    # PID controller for angular velocity
                    integral += delta * dt
                    derivative = (delta - previous_error) / dt
                    angular_velocity = Kp * delta + Ki * integral + Kd * derivative
                    previous_error = delta
                    
                    # Limit angular velocity
                    angular_velocity = max(min(angular_velocity, self.max_rotation), -self.max_rotation)
                    
                    # Move robot forward while adjusting its heading
                    linear_velocity = min(Kp * rho / self.scale, self.max_speed)
                    
                    self.theta += angular_velocity * dt
                    self.x += linear_velocity * math.cos(self.theta) * self.scale * dt
                    self.y += linear_velocity * math.sin(self.theta) * self.scale * dt
                    
                    # Update visualization
                    self.canvas.delete("robot")
                    self.draw()
                    root.update()
                    time.sleep(dt)
    
    def draw(self):
        """
        Draw the robot in the Tkinter canvas.
        """
        x1 = self.x - 35
        y1 = self.y - 35
        x2 = self.x + 35
        y2 = self.y + 35
        
        # Draw robot body
        self.canvas.create_oval(x1, y1, x2, y2, fill="blue", tags="robot")
        
        # Draw heading direction
        x_heading = self.x + 35 * math.cos(self.theta)
        y_heading = self.y + 35 * math.sin(self.theta)
        self.canvas.create_line(self.x, self.y, x_heading, y_heading, fill="red", width=3, tags="robot")
    
    def add_trajectory(self):
        """
        Generate a smooth trajectory (e.g., a sine wave or arc) and draw it.
        """
        self.canvas.delete("trajectory")  # Remove previous trajectory
        self.trajectory = [(self.x + i, self.y + 50 * math.sin(i / 50)) for i in range(0, 300, 10)]
        
        for i in range(len(self.trajectory) - 1):
            x1, y1 = self.trajectory[i]
            x2, y2 = self.trajectory[i + 1]
            self.canvas.create_line(x1, y1, x2, y2, fill="black", width=2, tags="trajectory")

def restart_simulation():
    """
    Restart simulation with the robot at a random position.
    """
    global epuck
    stop_robot()
    canvas.delete("all")
    x, y = random.randint(50, 550), random.randint(50, 550)
    epuck = EPuck2(canvas, x, y, random.uniform(-math.pi, math.pi))
    epuck.draw()

def start_move_to_point():
    """
    Start the move-to-point behavior.
    """
    goal_x, goal_y = 300, 200  # Goal Position in pixels
    canvas.create_oval(goal_x-5, goal_y-5, goal_x+5, goal_y+5, fill="red")  # Draw Goal Point
    epuck.move_to_point(goal_x, goal_y)

def start_follow_trajectory():
    """
    Start following the trajectory using PID control.
    """
    epuck.follow_trajectory()

def add_random_trajectory():
    """
    Add a smooth trajectory and update visualization.
    """
    epuck.add_trajectory()

def stop_robot():
    """
    Stop the robot's movement.
    """
    epuck.stop_movement()

# Initialize Tkinter Window
root = tk.Tk()
root.title("e-Puck 2 Robot Simulation")
canvas = tk.Canvas(root, width=600, height=600, bg="white")
canvas.pack()

# Create e-Puck robot
epuck = EPuck2(canvas, 100, 500, -math.pi/2)
epuck.draw()

# Buttons for different control algorithms
btn_move_to_point = tk.Button(root, text="Move to Point", command=start_move_to_point)
btn_move_to_point.pack()
btn_restart = tk.Button(root, text="Restart", command=restart_simulation)
btn_restart.pack()
btn_add_trajectory = tk.Button(root, text="Add Trajectory", command=add_random_trajectory)
btn_add_trajectory.pack()
btn_follow_trajectory = tk.Button(root, text="Follow Trajectory", command=start_follow_trajectory)
btn_follow_trajectory.pack()

root.mainloop()