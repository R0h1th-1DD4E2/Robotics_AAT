def sysCall_init():
    import math
    import threading
    import tkinter as tk
    sim = require('sim')

    global base_joint, elbow_joint, theta1_gui, theta2_gui, ee_position_var, L1, L2
    L1 = 1.2  # Link 1 length
    L2 = 1.2  # Link 2 length

    base_joint = sim.getObject('/Base/Base_joint')
    elbow_joint = sim.getObject('/Base/Base_joint/Elbow_joint')

    sim.setJointMode(base_joint, sim.jointmode_force, 0)
    sim.setJointMode(elbow_joint, sim.jointmode_force, 0)

    # Initialize joint angle globals
    theta1_gui = 0.0
    theta2_gui = 0.0

    # Placeholder for GUI EE display variable
    ee_position_var = None

    print("Initialized for Tkinter-based FK control")

    def launch_gui():
        global ee_position_var
        root = tk.Tk()
        root.title("2-Link Arm Angle Control (FK)")

        tk.Label(root, text="Base Joint θ₁ (degrees)").pack()
        slider1 = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, length=400)
        slider1.set(0)
        slider1.pack()

        tk.Label(root, text="Elbow Joint θ₂ (degrees)").pack()
        slider2 = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, length=400)
        slider2.set(0)
        slider2.pack()

        ee_position_var = tk.StringVar()
        ee_position_var.set("EE Position: (0.00, 0.00)")
        tk.Label(root, textvariable=ee_position_var, font=("Helvetica", 12)).pack(pady=10)

        def update_angles():
            import math
            global theta1_gui, theta2_gui
            theta1_gui = math.radians(slider1.get())
            theta2_gui = math.radians(slider2.get())
            root.after(100, update_angles)

        root.after(100, update_angles)
        root.mainloop()

    # Start the GUI in a new thread
    threading.Thread(target=launch_gui, daemon=True).start()


def sysCall_actuation():
    sim = require('sim')
    import math
    global theta1_gui, theta2_gui, ee_position_var, L1, L2

    # Apply joint angles
    sim.setJointTargetPosition(base_joint, theta1_gui)
    sim.setJointTargetPosition(elbow_joint, theta2_gui)

    # Forward Kinematics: compute EE position
    x = L1 * math.cos(theta1_gui) + L2 * math.cos(theta1_gui + theta2_gui)
    y = L1 * math.sin(theta1_gui) + L2 * math.sin(theta1_gui + theta2_gui)

    # Update EE position in GUI
    if ee_position_var is not None:
        ee_position_var.set(f"EE Position: ({x:.2f}, {y:.2f})")

    print(f"θ1 = {theta1_gui:.2f} rad, θ2 = {theta2_gui:.2f} rad | EE = ({x:.2f}, {y:.2f})")



