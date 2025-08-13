🤖 MuJoCo Tidybot Pick-and-Place Simulation

A robotic manipulation simulation implementing pick-and-place tasks using the Stanford Tidybot model in MuJoCo physics engine.
<p align="center">
  <img src="https://via.placeholder.com/600x400?text=Demo+GIF" alt="Demo GIF" />
  <br>
  <em>Tidybot performing pick-and-place operation</em>
</p>
✨ Features

🎯 Inverse Kinematics: Real-time joint angle calculation for target end-effector poses using SLSQP optimization
🎢 Smooth Trajectory Planning: Jerk-limited motion planning for 7-DOF arm using Ruckig library
🚗 Base Mobility Control: Keyboard teleoperation with arm stabilization
🤏 Force-based Grasp Detection: Contact force-based verification for reliable grasping
🖥️ Interactive GUI: Real-time 3D visualization with MuJoCo viewer

📋 Prerequisites

Python 3.8 or higher
Ubuntu 20.04+ / Windows 10+ / macOS 10.15+
OpenGL 3.3+ compatible graphics

🚀 Quick Start
Installation
bash# Clone the repository
git clone https://github.com/yourusername/mujoco-tidybot.git
cd mujoco-tidybot

# Install dependencies
pip install -r requirements.txt
Required packages:
bashpip install mujoco numpy scipy keyboard ruckig
Configuration

Update the model path in main.py:

python# main.py
XML_PATH = "path/to/your/stanford_tidybot/scene.xml"
Running the Simulation
bashpython main.py
🎮 Controls
KeyActionSpaceStart pick-and-place sequence8 / 5Move base forward / backward4 / 6Move base left / right7 / 9Rotate base counter-clockwise / clockwise2Stop base movementESCExit simulation
📁 Project Structure
mujoco-tidybot/
│
├── 📂 model/
│   └── stanford_tidybot/
│       └── scene.xml              # MuJoCo model definition
│
├── 📄 main.py                     # Entry point with space-bar trigger
├── 📄 simulation.py               # Main simulation orchestration
├── 📄 arm_controller.py           # Ruckig-based arm trajectory control
├── 📄 mobility_controller.py      # Base teleoperation & gravity compensation
├── 📄 ik_solver.py               # SLSQP-based inverse kinematics solver
├── 📄 grasp_checker.py           # Contact force-based grasp verification
├── 📄 requirements.txt           # Python dependencies
└── 📄 README.md                  # This file
🔧 Technical Details
System Architecture
mermaidgraph TD
    A[Main Entry] --> B[Simulation Manager]
    B --> C[Mobility Controller]
    B --> D[Arm Controller]
    B --> E[IK Solver]
    B --> F[Grasp Checker]
    C --> G[Keyboard Input]
    D --> H[Ruckig Trajectory]
    E --> I[SLSQP Optimizer]
    F --> J[Force Sensor]
Core Components
🦾 Arm Controller (arm_controller.py)

Implements computed-torque control with PD feedback
Uses Ruckig for jerk-limited trajectory generation
Manages 7-DOF arm with real-time torque computation

🚗 Mobility Controller (mobility_controller.py)

Decoupled base/arm control architecture
Gravity compensation during teleoperation
Thread-safe command buffer management

🧮 IK Solver (ik_solver.py)

SLSQP optimization for position/orientation error minimization
Joint limit enforcement
Real-time performance optimization

🤏 Grasp Checker (grasp_checker.py)

Contact force threshold detection (0.05 N)
Pad distance monitoring
Real-time grasp state feedback

Performance Specifications
ParameterValueControl Rate500 Hz (2 ms timestep)Arm Velocity Limit3 rad/sArm Acceleration Limit7 rad/s²Arm Jerk Limit150 rad/s³Grasp Force Threshold0.05 NPD Gains (Arm)Kp = 1500, Kd = 30
🔄 Workflow

Initialization: Load MuJoCo model and initialize controllers
Teleoperation Mode: Base control via keyboard while arm maintains pose
Pick Sequence (on Space key):

Calculate IK for approach position
Move to grasp pose
Close gripper and verify grasp
Lift object


Place Sequence:

Navigate to target location
Lower object
Open gripper
Return to home position


Resume Teleoperation: Return control to user

🛠️ Implementation Notes

Thread Safety: All mjData access is serialized with locks to prevent GUI/control loop conflicts
Real-time Control: Separate threads for mobility and arm control ensure smooth operation
Force Feedback: Contact forces are computed using MuJoCo's built-in contact solver
Trajectory Smoothness: Ruckig ensures C³ continuous trajectories (continuous jerk)

🐛 Troubleshooting
Common Issues

Model not found error
FileNotFoundError: XML file not found
Solution: Verify the XML_PATH in main.py points to the correct model file
Keyboard input not working
PermissionError: Keyboard access denied
Solution: Run with sudo on Linux or as administrator on Windows
Grasp detection failing

Adjust force threshold in grasp_checker.py
Check gripper pad collision geometry
Verify object mass and friction parameters



📊 Performance Optimization

Reduce viewer update frequency for better control performance
Adjust PD gains for specific robot configurations
Tune Ruckig limits based on actuator capabilities

🤝 Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

Fork the repository
Create your feature branch (git checkout -b feature/AmazingFeature)
Commit your changes (git commit -m 'Add some AmazingFeature')
Push to the branch (git push origin feature/AmazingFeature)
Open a Pull Request

📜 License
This project is licensed under the MIT License - see the LICENSE file for details.
🙏 Acknowledgments

Stanford Tidybot team for the robot model
MuJoCo development team for the physics engine
Ruckig library for trajectory generation

📮 Contact
For questions or support, please open an issue on GitHub or contact:

Email: nahoyong1@gmail.com
GitHub: @HoyongNa
