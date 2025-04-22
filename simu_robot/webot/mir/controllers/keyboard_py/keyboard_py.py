"""
Description: This controller is used to move the six-wheeled (2 actuated) robot MiR100 in an industrial environment
             using the keyboard. The keys are the following:
             
             vx         : ↑/↓
             ω          : ←/→
             Reset      : Space bar
             Pipe up    : a
"""
from controller import Robot, Supervisor, Motor, RangeFinder, Keyboard

# Constants
WHEEL_RADIUS = 0.0625
SPEED_MAX = 0.95
SPEED_MIN = -0.3
ANGULAR_SPEED_MAX = 0.3
ANGULAR_SPEED_MIN = -0.3
SPEED_INCREMENT = 0.05
ANGULAR_SPEED_INCREMENT = 0.05
DISTANCE_TO_CENTER = 0.2226
PIPE_UP_POSITION = 0.2  # Maximum height position for the pipe
PIPE_DOWN_POSITION = 0.0  # Minimum height position for the pipe
PIPE_MOVEMENT_SPEED = 0.05  # Speed at which the pipe moves

def main():
    # Initialize the robot
    robot = Supervisor()
    devant = robot.getFromDef('devant')
    translation_field = devant.getField('translation')
    # robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    
    # Get motor devices
    motor_left_wheel = robot.getDevice("middle_left_wheel_joint")
    motor_right_wheel = robot.getDevice("middle_right_wheel_joint")
    
    # Set motors to velocity control (position must be set to infinity)
    motor_left_wheel.setPosition(float('inf'))
    motor_right_wheel.setPosition(float('inf'))
    motor_left_wheel.setVelocity(0.0)
    motor_right_wheel.setVelocity(0.0)
    
    # Initialize target speeds
    target_speed = 0.0  # forwards speed [m]
    target_omega = 0.0  # angular speed [rad/s]
    
    # Set up depth camera
    depth_camera = robot.getDevice("depth_camera")
    depth_camera.enable(time_step)
    
    # Enable keyboard
    keyboard = robot.getKeyboard()
    keyboard.enable(time_step)
    
    # Print instructions
    print("To move the Mir100 with your keyboard, click first inside the simulation window and press:")
    print("  vx   : ↑/↓")
    print("  ω    : ←/→")
    print("  Reset: Space bar")
    print("  Pipe : a (toggle up/down)")
    
    # Main control loop
    while robot.step(time_step) != -1:
        key = keyboard.getKey()
        is_key_valid = True
        
        if key == Keyboard.UP:
            target_speed = target_speed + SPEED_INCREMENT
            target_speed = SPEED_MAX if target_speed > SPEED_MAX else target_speed
        elif key == Keyboard.DOWN:
            target_speed = target_speed - SPEED_INCREMENT
            target_speed = SPEED_MIN if target_speed < SPEED_MIN else target_speed
        elif key == Keyboard.LEFT:
            target_omega = target_omega + ANGULAR_SPEED_INCREMENT
            target_omega = ANGULAR_SPEED_MAX if target_omega > ANGULAR_SPEED_MAX else target_omega
        elif key == Keyboard.RIGHT:
            target_omega = target_omega - ANGULAR_SPEED_INCREMENT
            target_omega = ANGULAR_SPEED_MIN if target_omega < ANGULAR_SPEED_MIN else target_omega
        elif key == ord(' '):
            target_speed = 0
            target_omega = 0
        elif key == ord('A'):
            print("ok")
            new_value = [0.5, 0, 0]
            # translation_field.setSFVec3f(new_value)
                # Get current position
            current_position = translation_field.getSFVec3f()
            print(f"Current position of 'devant': {current_position}")
            
            # Add the new value to the current position
            updated_position = [current_position[0] + new_value[0],
                                current_position[1] + new_value[1],
                                current_position[2] + new_value[2]]
            
            # Set the updated position
            translation_field.setSFVec3f(updated_position)
            current_position = translation_field.getSFVec3f()
            # print(f"Position of 'devant': {current_position}")
            # print(devant.getField('Pose')) #here I want to get the postion of the object 
        else:
            is_key_valid = False
            
        if is_key_valid and key != ord('a'):
            print(f"vx:{target_speed:.2f}[m/s] ω:{target_omega:.2f}[rad/s]")
            # Compute wheel motor speeds from vx and ω
            motor_left_wheel.setVelocity((target_speed - target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS)
            motor_right_wheel.setVelocity((target_speed + target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS)

if __name__ == "__main__":
    main()