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
    # Create a table to store all the translation fields
    translation_fields = {}
    rotation_fields = {}
    
    # Initialize the robot
    robot = Supervisor()
    devant = robot.getFromDef('devant') #mir / bodyslot / devant
    translation_fields['devant'] = devant.getField('translation')
    rotation_fields['devant'] = devant.getField('rotation')
    
    devant2 = robot.getFromDef('devant2') 
    translation_fields['devant2'] = devant2.getField('translation')
    rotation_fields['devant2'] = devant2.getField('rotation')
    
    palette = robot.getFromDef('palettePrise') 
    translation_fields['palette'] = palette.getField('translation')
    rotation_fields['palette'] = palette.getField('rotation')

    print("Fields initialized:")
    for name, field in translation_fields.items():
        print(f"{name} translation: {field.getSFVec3f()}")
    for name, field in rotation_fields.items():
        print(f"{name} rotation: {field.getSFRotation()}")
    
    # Get robot and palette position
    robot_node = robot.getSelf()
    robot_translation_field = robot_node.getField('translation')
    robot_rotation_field = robot_node.getField('rotation')
    
    # Get initial positions and rotations
    palette_initial_position = translation_fields['palette'].getSFVec3f()
    robot_initial_position = robot_translation_field.getSFVec3f()
    
    palette_initial_rotation = rotation_fields['palette'].getSFRotation()
    robot_initial_rotation = robot_rotation_field.getSFRotation()
    
    # Calculate the relative offset between palette and robot
    palette_offset = [
        palette_initial_position[0] - robot_initial_position[0],
        palette_initial_position[1] - robot_initial_position[1],
        palette_initial_position[2] - robot_initial_position[2]
    ]
    
    # Calculate rotation difference (simplified - assuming main rotation is around y-axis)
    # This stores the initial relative rotation
    # Store robot and palette rotation in their entirety
    palette_rotation_offset = palette_initial_rotation.copy()
    robot_initial_rotation_copy = robot_initial_rotation.copy()
    
    # Track if the palette is raised and should follow the robot
    palette_raised = False
    
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
    print("  Raise Palette and Follow: A (toggle on)")
    print("  Lower Palette and Stop Following: E (toggle off)")
    
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
        # Raise palette and enable following when 'A' is pressed
        elif key == ord('A'):
            print("Raising palette and enabling following...")
            palette_translation = translation_fields['palette'].getSFVec3f()
            # Move up by 0.05 units
            palette_translation[2] += 0.05
            translation_fields['palette'].setSFVec3f(palette_translation)
            palette_raised = True
            print(f"Palette raised: {palette_translation}")
            
            # Recalculate both position and rotation offsets
            palette_initial_position = translation_fields['palette'].getSFVec3f()
            robot_initial_position = robot_translation_field.getSFVec3f()
            
            palette_initial_rotation = rotation_fields['palette'].getSFRotation()
            robot_initial_rotation = robot_rotation_field.getSFRotation()
            
            palette_offset = [
                palette_initial_position[0] - robot_initial_position[0],
                palette_initial_position[1] - robot_initial_position[1],
                palette_initial_position[2] - robot_initial_position[2]
            ]
            
            # Store robot and palette rotation in their entirety
            palette_rotation_offset = palette_initial_rotation.copy()
            robot_initial_rotation_copy = robot_initial_rotation.copy()
            
            print(f"New position offset: {palette_offset}")
            print(f"New rotation offset: {palette_rotation_offset}")

        # Lower palette and disable following when 'E' is pressed
        elif key == ord('E'):
            print("Lowering palette and disabling following...")
            palette_translation = translation_fields['palette'].getSFVec3f()
            # Move down by 0.05 units
            palette_translation[2] -= 0.05
            translation_fields['palette'].setSFVec3f(palette_translation)
            palette_raised = False
            print(f"Palette lowered: {palette_translation}")
        else:
            is_key_valid = False
        
        # Update palette position and rotation to follow robot if raised
        if palette_raised:
            # Update position
            current_robot_position = robot_translation_field.getSFVec3f()
            new_palette_position = [
                current_robot_position[0] + palette_offset[0],
                current_robot_position[1] + palette_offset[1],
                translation_fields['palette'].getSFVec3f()[2]  # Keep current Z height
            ]
            translation_fields['palette'].setSFVec3f(new_palette_position)
            
            # Update rotation - keep the full rotation values
            current_robot_rotation = robot_rotation_field.getSFRotation()
            # Simply apply the robot's rotation directly to the palette
            rotation_fields['palette'].setSFRotation(current_robot_rotation)
                
        if is_key_valid and key != ord('A') and key != ord('E'):
            print(f"vx:{target_speed:.2f}[m/s] ω:{target_omega:.2f}[rad/s]")
            # Compute wheel motor speeds from vx and ω
            motor_left_wheel.setVelocity((target_speed - target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS)
            motor_right_wheel.setVelocity((target_speed + target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS)

if __name__ == "__main__":
    main()