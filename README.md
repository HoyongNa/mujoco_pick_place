MuJoCo Tidybot Pick-and-Place Simulation
A robust robotic manipulation system implementing pick-and-place operations for the Stanford Tidybot in MuJoCo physics simulation.
Features

Inverse Kinematics (IK): Real-time joint angle computation for target end-effector poses
Smooth Trajectory Generation: Ruckig-based jerk-limited motion planning for 7-DOF arm
Base Mobility Control: Keyboard teleoperation with concurrent arm stabilization
Force-based Grasp Detection: Contact force monitoring for reliable object manipulation
Interactive GUI: Real-time 3D visualization with MuJoCo viewer

Requirements
bashpip install mujoco numpy scipy keyboard ruckig
Architecture
├── main.py                 # Entry point with space-bar trigger
├── simulation.py           # Core simulation orchestrator
├── arm_controller.py       # Torque-based arm control with Ruckig
├── mobility_controller.py  # Base teleoperation and arm gravity compensation
├── ik_solver.py           # Inverse kinematics solver
└── grasp_checker.py       # Contact-based grasp verification
Usage

Configure Model Path
python# main.py
XML_PATH = "path/to/stanford_tidybot/scene.xml"

Run Simulation
bashpython main.py

Controls

Space: Start pick-and-place sequence
8/5: Forward/backward base motion
4/6: Left/right base motion
7/9: Rotate base CCW/CW
2: Stop base
ESC: Exit simulation



Key Components
Arm Controller

Computed torque control with PD feedback
Optional disturbance observer (DOB)
Jerk-limited trajectories via Ruckig

Mobility Controller

Decoupled base and arm control
Gravity compensation during teleoperation
Thread-safe command buffer sharing

IK Solver

SLSQP optimization for 7-DOF redundancy
Position and orientation error minimization
Joint limit enforcement

Technical Details

Control Rate: Model timestep (typically 2ms)
Arm Limits: 3 rad/s velocity, 7 rad/s² acceleration, 150 rad/s³ jerk
Grasp Detection: 0.05N force threshold
PD Gains: Kp=1500, Kd=30 (arm tracking)

License
MIT