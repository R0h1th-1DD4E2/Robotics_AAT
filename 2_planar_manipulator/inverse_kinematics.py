def sysCall_init():
    import math
    sim = require('sim')
    
    global base_joint, elbow_joint, dummy_handle, target_box
    base_joint = sim.getObject('/Base/Base_joint')
    elbow_joint = sim.getObject('/Base/Base_joint/Elbow_joint')
    dummy_handle = sim.getObject('/Base/Base_joint/Elbow_joint/End_effector')
    target_box = sim.getObject('/Target_Box')  # Make sure this exists in the scene

    # Constants
    global L1, L2
    L1 = 1 + 0.2  # Length of first link (meters)
    L2 = 1 + 0.2  # Length of second link (meters)

    # Set joints to position control mode
    sim.setJointMode(base_joint, sim.jointmode_force, 0)
    sim.setJointMode(elbow_joint, sim.jointmode_force, 0)

    print("Initialized with FK and IK support")

def sysCall_actuation():
    import math
    sim = require('sim')

    # Get joint angles
    theta1 = sim.getJointPosition(base_joint)
    theta2 = sim.getJointPosition(elbow_joint)

    # Forward Kinematics: Compute current EE position
    x_fk = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y_fk = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

    # Target Position (from box)
    box_pos = sim.getObjectPosition(target_box, -1)
    x_target = box_pos[0] - 0.1  # Adjust based on target position
    y_target = box_pos[1] - 0.1

    # Compute error
    dx = x_target - x_fk
    dy = y_target - y_fk

    # Threshold for stopping condition (small enough error)
    threshold = 0.05  # Acceptable error in meters

    if abs(dx) < threshold and abs(dy) < threshold:
        print("Target reached.")
        return  # Stop once the target is reached

    # Small gain to scale movement
    alpha = 0.5  # You can adjust this for faster/slower movement
    dx *= alpha
    dy *= alpha

    # Jacobian matrix
    J11 = -L1 * math.sin(theta1) - L2 * math.sin(theta1 + theta2)
    J12 = -L2 * math.sin(theta1 + theta2)
    J21 = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    J22 = L2 * math.cos(theta1 + theta2)

    # Compute determinant
    det = J11 * J22 - J12 * J21
    # Apply damping factor if the d)  # Adjust for the negative y-axiseterminant is too small
    if abs(det) < 1e-4:  # Increased threshold
        print("Warning: Jacobian determinant is small, applying damping.")
        det += 1e-4  # Adding a small value to prevent division by a very small number

    if abs(det) < 1e-5:
        print("Jacobian is near-singular.")
        return

    # Inverse Jacobian (2x2)
    invJ11 = J22 / det
    invJ12 = -J12 / det
    invJ21 = -J21 / det
    invJ22 = J11 / det

    # Compute joint velocities
    dtheta1 = invJ11 * dx + invJ12 * dy
    dtheta2 = invJ21 * dx + invJ22 * dy

    # Time step
    dt = sim.getSimulationTimeStep()

    # Update joint angles
    theta1_new = theta1 + dtheta1 * dt 
    theta2_new = theta2 + dtheta2 * dt

    # Apply new positions
    sim.setJointTargetPosition(base_joint, theta1_new)
    sim.setJointTargetPosition(elbow_joint, theta2_new)

    # Debugging output
    print('Tracking Target: x = {:.2f}, y = {:.2f} | Error dx = {:.2f}, dy = {:.2f}'.format(x_target, y_target, dx, dy))

