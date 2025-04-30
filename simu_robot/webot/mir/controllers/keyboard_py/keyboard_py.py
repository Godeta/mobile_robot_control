from controller import Robot, Supervisor, Motor, RangeFinder, Keyboard
import math

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
    
    # Get references to nodes
    palette_node = robot.getFromDef('palettePrise')
    carton_node = robot.getFromDef('cartonPrise')
    robot_node = robot.getSelf()
    
    # Check if all nodes are valid
    if not palette_node or not robot_node:
        print("Error: Could not find all required nodes")
        return
    
    # Get fields for robot and palette
    palette_translation_field = palette_node.getField('translation')
    palette_rotation_field = palette_node.getField('rotation')
    carton_translation_field = carton_node.getField('translation')
    carton_rotation_field = carton_node.getField('rotation')
    robot_translation_field = robot_node.getField('translation')
    robot_rotation_field = robot_node.getField('rotation')
    
    # Track if the palette is raised and should follow the robot
    palette_raised = False
    
    # Variables to store the relative position and orientation
    palette_offset = [0, 0, 0]
    
    # Get time step
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
    print("  Raise Palette and Follow Robot: A")
    print("  Lower Palette and Stop Following: E")
    
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
        # Raise palette and start following when 'A' is pressed
        elif key == ord('A'):
            try:
                print("Raising palette and enabling following...")
                
                # First raise the palette a bit
                current_pos = palette_translation_field.getSFVec3f()
                new_pos = [current_pos[0], current_pos[1], current_pos[2] + 0.05]
                palette_translation_field.setSFVec3f(new_pos)
                current_pos2 = carton_translation_field.getSFVec3f()
                new_pos = [current_pos2[0], current_pos2[1], current_pos2[2] + 0.05]
                carton_translation_field.setSFVec3f(new_pos)
                
                # Calculate the offset between palette and robot at this moment
                robot_pos = robot_translation_field.getSFVec3f()
                palette_pos = palette_translation_field.getSFVec3f()
                carton_pos = carton_translation_field.getSFVec3f()
                
                # Transform to robot's local coordinates
                robot_rot = robot_rotation_field.getSFRotation()
                # angle = robot_rot[3]  # Assuming rotation around Y-axis
                
                # Calculate relative position in robot's local space
                dx = palette_pos[0] - robot_pos[0]
                dy = palette_pos[1] - robot_pos[1]
                dz = palette_pos[2] - robot_pos[2]
                
                # Store the offset in robot's local space
                palette_offset = [dx, dy, dz]
                
                dx = carton_pos[0] - robot_pos[0]
                dy = carton_pos[1] - robot_pos[1]
                dz = carton_pos[2] - robot_pos[2]
                carton_offset = [dx, dy, dz]
                
                palette_raised = True
                print(f"Palette raised and following enabled. Offset: {palette_offset}")
            except Exception as e:
                print(f"Error raising palette: {e}")
            
        # Lower palette and stop following when 'E' is pressed
        elif key == ord('E'):
            try:
                if palette_raised:
                    print("Lowering palette and disabling following...")
                    
                    # Lower the palette a bit
                    current_pos = palette_translation_field.getSFVec3f()
                    new_pos = [current_pos[0], current_pos[1], current_pos[2] - 0.05]
                    palette_translation_field.setSFVec3f(new_pos)
                    current_pos2 = carton_translation_field.getSFVec3f()
                    new_pos = [current_pos2[0], current_pos2[1], current_pos2[2] - 0.05]
                    carton_translation_field.setSFVec3f(new_pos)
                    
                    palette_raised = False
                    print("Palette lowered and following disabled")
            except Exception as e:
                print(f"Error lowering palette: {e}")
        else:
            is_key_valid = False
                
        # Update palette position if following is enabled
        if palette_raised:
            try:
                # Get current robot position and rotation
                robot_pos = robot_translation_field.getSFVec3f()
                robot_rot = robot_rotation_field.getSFRotation()
                
                # Extract rotation angle (assuming Y-axis rotation which is common for ground vehicles)
                # angle = robot_rot[3]
                axis = [robot_rot[0], robot_rot[1], robot_rot[2]]
                
                # Calculate new position based on robot position and offset
                # For simplicity, we'll just consider Y-axis rotation (most common for ground vehicles)
                # For full 3D rotation, you would need to use rotation matrices or quaternions
                
                # If rotation is primarily around Y-axis
                # if abs(axis[1]) > 0.9:  # Y-axis rotation
                #     # Apply rotation to the X-Z offset
                #     rotated_x = palette_offset[0] * math.cos(angle) - palette_offset[2] * math.sin(angle)
                #     rotated_z = palette_offset[0] * math.sin(angle) + palette_offset[2] * math.cos(angle)
                    
                #     new_pos = [
                #         robot_pos[0] + rotated_x,
                #         robot_pos[1] + palette_offset[1],  # Y offset remains the same
                #         robot_pos[2] + rotated_z
                #     ]
                # else:
                    # Fallback for other rotation axes - simplified
                new_pos = [
                    robot_pos[0] + palette_offset[0],
                    robot_pos[1] + palette_offset[1],
                    robot_pos[2] + palette_offset[2]
                ]
                new_pos2 = [
                    robot_pos[0] + carton_offset[0],
                    robot_pos[1] + carton_offset[1],
                    robot_pos[2] + carton_offset[2]
                ]
                
                # Update palette position
                palette_translation_field.setSFVec3f(new_pos)
                carton_translation_field.setSFVec3f(new_pos2)
                # Set palette rotation to match robot rotation
                # palette_rotation_field.setSFRotation(robot_rot)
                
            except Exception as e:
                print(f"Error updating palette position: {e}")
                
        if is_key_valid and key != ord('A') and key != ord('E'):
            print(f"vx:{target_speed:.2f}[m/s] ω:{target_omega:.2f}[rad/s]")
            # Compute wheel motor speeds from vx and ω
            motor_left_wheel.setVelocity((target_speed - target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS)
            motor_right_wheel.setVelocity((target_speed + target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS)

if __name__ == "__main__":
    main()