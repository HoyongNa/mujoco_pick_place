# MuJoCo Dual-Robot Navigation & Pick-and-Place System

A comprehensive robotics simulation system built on MuJoCo physics engine featuring dual-robot control, autonomous navigation with LIDAR mapping, pick-and-place manipulation, and natural language command execution via LLM integration.

## 📋 Documentation

### For Ubuntu 22.04 Users

- **🚀 [Quick Start Guide](QUICKSTART_UBUNTU.md)** - Get running in 5 minutes with one command
- **📖 [Complete Installation Guide](README_UBUNTU_22.04.md)** - Detailed instructions for ACADOS, RoboCasa, Ruckig, and more
- **⚡ [Automated Installer Script](install_ubuntu_22.04.sh)** - One-command installation

**New to this project on Ubuntu?** → Start with the **[Quick Start Guide](QUICKSTART_UBUNTU.md)**

---

## Overview

This system simulates two Stanford Tidybot robots operating simultaneously in a shared environment. Each robot can navigate autonomously, perform pick-and-place tasks, and respond to natural language commands. The system features centralized physics stepping for thread-safe operation and supports both keyboard teleoperation and LLM-driven autonomous operation.

## Key Features

### Multi-Robot System
- **Dual Robot Control**: Simultaneous operation of two independent robots (Robot1 - White, Robot2 - Blue)
- **Independent Control**: Each robot has separate arm controllers, path planners, and mobility controllers
- **Parallel Execution**: Both robots can execute tasks simultaneously without interference

### Navigation & Mapping
- **LIDAR-based Mapping**: Real-time occupancy grid mapping using simulated LIDAR sensor
- **A* Path Planning**: Efficient path planning with obstacle avoidance
- **MPC Hybrid Controller**: Model Predictive Control for smooth path following
- **Waypoint Generation**: Automatic waypoint interpolation for navigation

### Manipulation
- **7-DOF Arm Control**: Precise arm control with torque-based controllers
- **Inverse Kinematics**: IK solver for end-effector positioning
- **Grasp Detection**: Force-based grasp verification
- **Pick-and-Place Tasks**: Automated object manipulation with collision checking

### LLM Integration
- **Natural Language Commands**: Control robots using plain English via GPT-4o
- **Task Planning**: Automatic task decomposition and optimization
- **Multi-Robot Coordination**: Execute commands on individual robots or both simultaneously
- **Command Syntax**:
  - `robot1 <command>` - Execute on Robot1 only
  - `robot2 <command>` - Execute on Robot2 only
  - `both <command>` - Execute on both robots simultaneously
  - `<command>` - Default to Robot1

### Thread-Safe Architecture
- **Centralized Stepping**: All physics steps execute in the main thread
- **Command Queue System**: Thread-safe command handling for LLM operations
- **Lock-based Synchronization**: Prevents race conditions in multi-threaded operations

## Requirements

### Core Dependencies
```
numpy>=1.21.0
mujoco>=2.3.0
opencv-python>=4.5.0
matplotlib>=3.3.0
scipy>=1.7.0
casadi>=3.5.5
pynput>=1.7.0
Pillow>=8.0.0
networkx>=2.6.0
```

### Optional Dependencies
```
openai>=1.0.0      # For LLM integration
pytest>=6.0.0      # For testing
ipython>=7.0.0     # For development
jupyter>=1.0.0     # For notebooks
```

### Advanced Dependencies (Ubuntu 22.04)

For full functionality including ACADOS MPC and RoboCasa kitchen environments, see:
- **[Ubuntu 22.04 Installation Guide](README_UBUNTU_22.04.md)** - Complete setup with ACADOS, RoboCasa, and Ruckig

## Installation

### Quick Installation (Basic Dependencies Only)

```bash
# Install Python dependencies
pip install -r requirements.txt

# Set up OpenAI API key (optional)
export OPENAI_API_KEY="your-api-key-here"
```

### Full Installation (Ubuntu 22.04 with ACADOS + RoboCasa)

**Option 1: Automated Installation (Recommended)**
```bash
# Run the automated installer
chmod +x install_ubuntu_22.04.sh
./install_ubuntu_22.04.sh
```

**Option 2: Manual Installation**

Follow the detailed guide: **[README_UBUNTU_22.04.md](README_UBUNTU_22.04.md)**

This includes:
- System dependencies
- MuJoCo installation
- ACADOS MPC framework
- RoboCasa kitchen environments
- Optional Ruckig trajectory generation
- Troubleshooting guide

## Usage

### Starting the System

Run the main simulation:
```bash
python main.py
```

The system will:
1. Load the dual-robot scene from `model/stanford_tidybot/scene_dual_robot.xml`
2. Initialize LIDAR mapping and path planners for both robots
3. Start the MuJoCo viewer with 3D visualization
4. Begin in keyboard teleoperation mode

### Keyboard Controls

#### Robot1 (White)
- **W/A/S/D**: Move forward/left/backward/right
- **Q/E**: Rotate left/right
- **C**: Stop
- **F8-F11**: Navigate to rooms 1-4 (NW, NE, SW, SE)

#### Robot2 (Blue)
- **Numpad 8/4/5/6**: Move forward/left/backward/right
- **Numpad 7/9**: Rotate left/right
- **Numpad 2**: Stop
- **F4-F7**: Navigate to rooms 1-4 (NW, NE, SW, SE)

#### Common
- **- (Minus)**: Toggle between keyboard and LLM mode
- **ESC**: Exit the simulation

### LLM Mode

Press `-` to enter LLM mode and control robots with natural language:

```bash
# Single robot commands
robot1 go to room 1
robot2 pick up the apple
robot1 move forward 2 meters

# Multi-robot commands
both move to room 2
both pick up the nearest object

# Default (Robot1)
navigate to room 3
pick up the red block
```

The system will:
1. Parse your command using GPT-4o
2. Generate a task plan with subtasks
3. Display the execution plan
4. Execute tasks autonomously
5. Provide progress updates

Press `-` again to return to keyboard mode.

## Project Structure

```
code2/
├── main.py                          # Main entry point and system orchestration
├── README.md                        # This file - project overview
├── README_UBUNTU_22.04.md           # Detailed Ubuntu 22.04 installation guide
├── QUICKSTART_UBUNTU.md             # Quick start guide for Ubuntu
├── install_ubuntu_22.04.sh          # Automated installation script
├── config/
│   ├── constants.py                 # System constants and parameters
│   ├── robot_config.py              # Robot configuration
│   └── stabilization_config.py      # Stabilization parameters
├── controllers/
│   ├── arm/
│   │   ├── arm_controller.py        # Arm trajectory controller
│   │   ├── arm_holder.py            # Arm holding controller
│   │   ├── torque_controller.py     # Low-level torque control
│   │   ├── eso.py                   # Extended State Observer
│   │   └── trajectory_initializer.py # Trajectory initialization
│   ├── base/
│   │   ├── base_torque_controller.py    # Base torque control
│   │   ├── base_velocity_teleop.py      # Velocity-based teleoperation
│   │   ├── velocity_keyboard_handler.py # Keyboard input handler
│   │   └── velocity_mobility_controller.py # Mobility control
│   └── gripper/
│       └── grasp_checker.py         # Grasp force verification
├── lidar_mapping/
│   ├── lidar_sensor.py              # LIDAR sensor simulation
│   ├── occupancy_grid.py            # Occupancy grid representation
│   ├── mapping_system.py            # Mapping coordination
│   └── visualizer.py                # Map visualization
├── path_planning/
│   ├── astar_planner.py             # A* path planning algorithm
│   ├── mpc_hybrid_controller.py     # MPC-based path following
│   ├── map_processor.py             # Map preprocessing
│   └── waypoint_interpolation.py    # Waypoint generation
├── tasks/
│   ├── waypoint_generator.py        # Task waypoint generation
│   └── feasibility_checker.py       # Task feasibility validation
├── llm_planner/
│   ├── planner/
│   │   ├── planner.py               # LLM-based task planner
│   │   ├── scene_parser.py          # Scene understanding
│   │   └── task_types.py            # Task type definitions
│   └── executor/
│       ├── base.py                  # Task executor base
│       ├── handlers/                # Task-specific handlers
│       │   ├── navigate.py          # Navigation tasks
│       │   ├── pick.py              # Pick tasks
│       │   ├── place.py             # Place tasks
│       │   └── wait.py              # Wait tasks
│       ├── status.py                # Execution status tracking
│       └── result.py                # Execution results
├── kinematics/
│   └── ik_solver.py                 # Inverse kinematics solver
├── simulation/
│   ├── simulation_manager.py        # Physics simulation manager
│   ├── viewer_manager.py            # MuJoCo viewer management
│   └── central_step_manager.py      # Centralized step coordination
├── model/
│   └── stanford_tidybot/
│       ├── scene_dual_robot.xml     # Dual robot scene definition
│       ├── tidybot.xml              # Robot1 model
│       └── tidybot_robot2.xml       # Robot2 model
├── robocasa_integration.py          # RoboCasa kitchen integration
├── generate_and_compile.py          # ACADOS MPC code generator
└── scripts/
    └── [various utility scripts]
```

## Architecture

### Centralized Stepping
All physics simulation steps are executed in the main thread to ensure thread safety with MuJoCo. Other threads (input handlers, LLM executors) use command queues and callbacks to request operations.

### Component Flow
1. **Input Layer**: Keyboard listener or LLM input thread captures user commands
2. **Command Queue**: Thread-safe queue stores pending commands
3. **Main Loop**: Processes commands, updates controllers, and steps physics
4. **Controllers**: Execute control laws (mobility, arm, path following)
5. **Simulation**: MuJoCo physics engine updates world state
6. **Visualization**: Viewer renders current state with overlays

### Multi-Robot Design
- Each robot has independent controller instances
- Shared simulation manager coordinates physics stepping
- Arm busy flags prevent control conflicts
- Separate LLM executors enable parallel task execution

## Room Layout

The default environment has four predefined rooms:

```
Room 1 (NW): (-2.0,  2.0)
Room 2 (NE): ( 2.0,  2.0)
Room 3 (SW): (-2.0, -2.0)
Room 4 (SE): ( 2.0, -2.0)
```

Navigate to rooms using F-keys (F8-F11 for Robot1, F4-F7 for Robot2).

## Advanced Features

### Extended State Observer (ESO)
The arm controller uses an ESO to estimate and compensate for disturbances, improving tracking performance and rejecting external forces.

### MPC Hybrid Controller
Model Predictive Control optimizes control inputs over a prediction horizon, ensuring smooth path following while respecting velocity and acceleration constraints.

### Task Plan Optimization
The LLM planner can merge sequential navigation tasks, reorder operations for efficiency, and split multi-robot plans into parallel execution.

### RoboCasa Kitchen Environments
Full kitchen environments with cabinets, counters, appliances, and realistic object physics. Multiple kitchen layouts and styles available. See [README_UBUNTU_22.04.md](README_UBUNTU_22.04.md) for installation.

## Troubleshooting

### Quick Fixes

**Map Not Found**: The map will be auto-generated on first run
**LLM Module Unavailable**: Set `OPENAI_API_KEY` environment variable
**Graphics Issues**: Try `export MUJOCO_GL=osmesa` for headless systems

### Detailed Troubleshooting

For comprehensive troubleshooting including ACADOS, RoboCasa, and system-specific issues, see:
**[Ubuntu 22.04 Troubleshooting Guide](README_UBUNTU_22.04.md#troubleshooting)**

## Development

### Running Tests
```bash
pytest
```

### Viewing Saved Maps
```bash
python scripts/view_saved_map.py
```

### Generating ACADOS MPC Code
```bash
python generate_and_compile.py
```

### Testing RoboCasa Integration
```bash
python robocasa_integration.py --layout G-shaped --style modern --physics-mode balanced
```

## Credits

- **Physics Engine**: MuJoCo (DeepMind)
- **Robot Model**: Stanford TidyBot
- **MPC Framework**: ACADOS
- **Kitchen Environments**: RoboCasa
- **LLM Integration**: OpenAI GPT-4o
- **Path Planning**: A* algorithm, MPC via CasADi

## License

[Add your license information here]

## Additional Resources

- **[Quick Start (Ubuntu 22.04)](QUICKSTART_UBUNTU.md)**
- **[Full Installation Guide (Ubuntu 22.04)](README_UBUNTU_22.04.md)**
- **[MuJoCo Documentation](https://mujoco.readthedocs.io/)**
- **[ACADOS Documentation](https://docs.acados.org/)**
- **[RoboCasa GitHub](https://github.com/robocasa/robocasa)**
