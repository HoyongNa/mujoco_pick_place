# MuJoCo Dual-Robot Navigation & Pick-and-Place System

A sophisticated dual-robot simulation system featuring advanced control algorithms for mobile manipulation in kitchen environments using MuJoCo physics engine.

## 🌟 Features

### Advanced Control Systems
- **ACADOS MPC Navigation**: Model Predictive Control for optimal path following
- **ESO-DOB Cascade Control**: Extended State Observer with Disturbance Observer for precise arm control
- **A* Path Planning**: Efficient pathfinding with dynamic obstacle avoidance
- **LLM-Based Task Planning**: Natural language command interpretation using GPT-4o

### Dual-Robot Coordination
- **Robot1 (TidyBot)**: Primary mobile manipulator
- **Robot2**: Secondary robot for collaborative tasks
- **Collision Avoidance**: Real-time collision detection and avoidance
- **Coordinated Pick-and-Place**: Multi-robot task execution

### RoboCasa Integration
- **Realistic Kitchen Environments**: G-shaped, U-shaped, L-shaped layouts
- **Dynamic Object Spawning**: Random object placement for training
- **Multiple Design Styles**: Modern, industrial, scandinavian, coastal, traditional
- **Fixture Generation**: Realistic kitchen appliances and furniture

### Real-Time Capabilities
- **LIDAR Mapping**: Dynamic environment mapping
- **Visual Servoing**: Camera-based manipulation control
- **Thread-Safe Simulation**: Parallel processing for real-time performance
- **Performance Monitoring**: Comprehensive diagnostics and logging

---

## 📋 Table of Contents

- [Installation](#installation)
  - [Quick Installation](#quick-installation-3-steps)
  - [Detailed Installation](#detailed-installation)
  - [Post-Installation Steps](#post-installation-steps)
  - [Platform-Specific Instructions](#platform-specific-instructions)
- [RoboCasa Kitchen Environments](#robocasa-kitchen-environments)
- [ACADOS MPC Setup](#acados-mpc-setup)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)

---

## Installation

This section contains complete installation instructions for all platforms and configurations.

### Quick Installation (3 Steps)

```bash
# Step 1: Install Python dependencies
cd ~/mujoco_pick_place
pip install -r requirements.txt --break-system-packages

# Step 2: Generate ACADOS MPC code (required for control)
python3 generate_and_compile.py

# Step 3: Run the simulation
python3 main.py
```

**Optional**: Install RoboCasa for kitchen environments (see [RoboCasa section](#robocasa-kitchen-environments))

---

### Detailed Installation

#### Prerequisites

- **Python**: 3.8 or higher
- **Operating System**: Ubuntu 22.04, Ubuntu 20.04, Windows 10/11, or macOS
- **GPU**: Recommended for visualization (optional)
- **RAM**: Minimum 8GB, recommended 16GB
- **Storage**: ~500MB for core dependencies, ~1GB with RoboCasa

#### Installation Methods

##### Method 1: Virtual Environment (Recommended)

Best for development and isolated environments:

```bash
# Create virtual environment
cd ~/mujoco_pick_place
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # Linux/macOS
# or
venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt

# Install development tools (optional)
pip install -r requirements-dev.txt
```

**Benefits:**
- ✅ Isolated from system packages
- ✅ No permission issues
- ✅ Easy to reset if needed
- ✅ Multiple Python versions possible

##### Method 2: System-Wide Installation (Ubuntu)

For system-wide access:

```bash
pip install -r requirements.txt --break-system-packages
```

**Note**: Requires `--break-system-packages` flag on Ubuntu 22.04+

##### Method 3: User Installation

Install in user directory without root:

```bash
pip install -r requirements.txt --user
```

#### Core Dependencies

The following packages will be installed:

| Package | Version | Purpose |
|---------|---------|---------|
| numpy | >=1.21.0, <2.0.0 | Numerical computing |
| mujoco | >=2.3.0, <=3.2.6 | Physics simulation |
| scipy | >=1.7.0, <1.15.0 | Scientific computing |
| opencv-python | >=4.5.0 | Computer vision |
| matplotlib | >=3.3.0 | Visualization |
| casadi | >=3.5.5 | Optimization |
| networkx | >=2.6.0 | Graph algorithms |
| pynput | >=1.7.0 | Keyboard input |
| Pillow | >=8.0.0 | Image processing |
| pyyaml | >=5.4.0 | Configuration files |
| h5py | >=3.1.0 | Data file handling |
| tqdm | >=4.60.0 | Progress bars |
| termcolor | >=1.1.0 | Colored output |
| imageio | >=2.9.0 | Image I/O |

#### Optional Dependencies

```bash
# LLM Integration (for natural language control)
pip install openai>=1.0.0

# Development Tools
pip install pytest ipython jupyter

# Or install all development tools
pip install -r requirements-dev.txt
```

---

### Post-Installation Steps

#### 1. Generate ACADOS MPC Code (Required)

The `generate_and_compile.py` script creates optimized C code for Model Predictive Control:

```bash
python3 generate_and_compile.py
```

**What it does:**
- Generates optimized C solver code for MPC
- Creates `c_generated_code/` directory
- Compiles solver for your platform
- Generates Python interface

**What's in `c_generated_code/`:**
- Compiled solver libraries (.so, .dll, .dylib)
- MPC configuration files
- Python wrapper code
- ACADOS problem formulation

**When to regenerate:**
- ✅ After cloning the repository (required!)
- ✅ After modifying MPC parameters
- ✅ When switching between different machines/OS
- ✅ If you see MPC-related import errors

**Why excluded from git:**
- Platform-specific (compiled for your OS)
- Large file size (~10-50 MB)
- Can be regenerated easily

**Troubleshooting:**
```bash
# If generation fails, check ACADOS installation
python3 -c "from acados_template import AcadosOcp; print('✅ ACADOS OK')"

# Check path
echo $ACADOS_SOURCE_DIR  # Should point to ACADOS installation
```

#### 2. Set Up OpenAI API Key (Optional)

For LLM-based natural language control:

```bash
# Linux/macOS
export OPENAI_API_KEY="your-api-key-here"

# Or add to ~/.bashrc for persistence
echo 'export OPENAI_API_KEY="your-api-key-here"' >> ~/.bashrc

# Windows (PowerShell)
$env:OPENAI_API_KEY="your-api-key-here"

# Windows (CMD)
set OPENAI_API_KEY=your-api-key-here
```

---

### Platform-Specific Instructions

#### Ubuntu 22.04 / 24.04

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python and pip
sudo apt install python3 python3-pip python3-venv -y

# Install system dependencies
sudo apt install build-essential cmake git libgl1-mesa-glx libglib2.0-0 -y

# Clone repository
git clone https://github.com/HoyongNa/mujoco_pick_place.git
cd mujoco_pick_place

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt

# Generate ACADOS code
python3 generate_and_compile.py

# Run simulation
python3 main.py
```

**Ubuntu-Specific Notes:**
- Use `--break-system-packages` for system-wide installs
- Virtual environment recommended to avoid system package conflicts
- For headless servers: `export MUJOCO_GL=osmesa`

#### Windows 10/11

```powershell
# Install Python from python.org (3.8+)
# Install Git from git-scm.com

# Install Visual C++ Build Tools (required for some packages)
# Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/

# Clone repository
git clone https://github.com/HoyongNa/mujoco_pick_place.git
cd mujoco_pick_place

# Create virtual environment
python -m venv venv
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Generate ACADOS code
python generate_and_compile.py

# Run simulation
python main.py
```

**Windows-Specific Notes:**
- Requires Visual C++ Build Tools for compilation
- Use PowerShell or CMD
- GPU drivers recommended for visualization

#### macOS

```bash
# Install Homebrew if not installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python
brew install python@3.10

# Clone repository
git clone https://github.com/HoyongNa/mujoco_pick_place.git
cd mujoco_pick_place

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip3 install -r requirements.txt

# Generate ACADOS code
python3 generate_and_compile.py

# Run simulation
python3 main.py
```

**macOS-Specific Notes:**
- Use Homebrew Python (not system Python)
- May require Xcode Command Line Tools: `xcode-select --install`

---

### Verification

After installation, verify everything works:

```bash
# Check Python version
python3 --version  # Should be 3.8+

# Check core dependencies
python3 << EOF
import numpy
import mujoco
import cv2
import matplotlib
import scipy
import casadi
import yaml
import h5py
print("✅ All core dependencies installed!")
print(f"NumPy: {numpy.__version__}")
print(f"MuJoCo: {mujoco.__version__}")
EOF

# Check ACADOS code generation
ls -la c_generated_code/  # Should contain compiled files

# Test run (should open visualization)
python3 main.py
```

**Expected output:**
```
[SimulationManager] Initializing dual-robot simulation system
[SimulationManager] Robot1: 11 DOF (3 base + 8 arm)
[SimulationManager] Robot2: 11 DOF (3 base + 8 arm)
[Controller] ACADOS MPC initialized
[Controller] ESO-DOB cascade control initialized
[SimulationManager] ✅ Simulation ready!
```

---

## RoboCasa Kitchen Environments

**Note**: The `robocasa_integration.py` file in the project root is **required** to use RoboCasa kitchen environments. This file handles the integration between your robots and the RoboCasa kitchen scenes.

### Why RoboCasa?

- **Realistic Environments**: Photo-realistic kitchen layouts
- **Diverse Scenarios**: Multiple kitchen designs and styles
- **Object Datasets**: Thousands of kitchen objects
- **Training Data**: For imitation learning and reinforcement learning

### Quick RoboCasa Installation

```bash
# Step 1: Install robosuite (CRITICAL - missing dependency!)
pip install robosuite --break-system-packages

# Step 2: Clone RoboCasa into your project
cd ~/mujoco_pick_place
git clone https://github.com/robocasa/robocasa.git

# Step 3: Install robocasa
cd robocasa
pip install -e . --break-system-packages

# Step 4: Verify installation
python3 -c "from robocasa.environments.kitchen.kitchen import Kitchen; print('✅ RoboCasa Success!')"
```

### Automated RoboCasa Installation

We provide helper scripts to make installation easier:

#### Option 1: Interactive Diagnostic Tool

```bash
python3 scripts/installation/fix_robocasa_deps.py
```

**Features:**
- Checks all dependencies
- Shows what's missing
- Offers to install missing packages
- Provides detailed diagnostics
- Step-by-step guidance

#### Option 2: Automated Installation Script

```bash
bash scripts/installation/install_robocasa_complete.sh
```

**What it does:**
- Installs robosuite automatically
- Clones and installs robocasa
- Verifies both packages
- Tests Kitchen class import
- Reports success/failure

### RoboCasa Verification

Run these commands to verify your RoboCasa installation:

```bash
# Check robosuite
python3 -c "import robosuite; print('✅ robosuite:', robosuite.__version__)"

# Check robocasa
python3 -c "import robocasa; print('✅ robocasa:', robocasa.__version__)"

# Check Kitchen class
python3 -c "from robocasa.environments.kitchen.kitchen import Kitchen; print('✅ Kitchen class OK')"

# Check KitchenArena
python3 -c "from robocasa.models.scenes.kitchen_arena import KitchenArena; print('✅ KitchenArena OK')"
```

### Common Warnings (Safe to Ignore)

These warnings are normal and won't affect functionality:

- ⚠️ "No private macro file found" - Optional feature
- ⚠️ "Could not import robosuite_models" - Optional models
- ⚠️ "Could not load the mink-based whole-body IK" - Optional IK solver
- ⚠️ "mimicgen environments not imported" - Optional package

### Testing RoboCasa Integration

Once installed, test the integration with different configurations:

```bash
# Test with G-shaped kitchen (default)
python3 robocasa_integration.py --layout G-shaped --style modern

# Test with U-shaped kitchen
python3 robocasa_integration.py --layout U-shaped --style industrial

# Test with different physics modes
python3 robocasa_integration.py --physics-mode balanced
```

**Available Options:**

| Parameter | Options | Description |
|-----------|---------|-------------|
| `--layout` | G-shaped, U-shaped, L-shaped, galley, one-wall | Kitchen layout |
| `--style` | modern, industrial, scandinavian, coastal, traditional | Design style |
| `--physics-mode` | fast, balanced, accurate | Physics quality |

**Physics Modes:**
- **fast**: Development mode, less accurate but faster
- **balanced**: Recommended for most use cases
- **accurate**: Precise physics, slower simulation

### RoboCasa Troubleshooting

#### Error: ModuleNotFoundError: No module named 'robocasa.environments'

**Solution:**
```bash
# Install robosuite first (it's a missing dependency)
pip install robosuite --break-system-packages

# Then install robocasa
cd ~/mujoco_pick_place/robocasa
pip install -e . --break-system-packages

# Verify
python3 -c "from robocasa.environments.kitchen.kitchen import Kitchen; print('✅')"
```

#### Error: "Invalid username or token" when cloning

**Solution:**
```bash
# Option 1: Use SSH
git clone git@github.com:robocasa/robocasa.git

# Option 2: Configure git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Option 3: Use HTTPS with token
# Create token at: https://github.com/settings/tokens
```

#### Error: NumPy version incompatibility

**Solution:**
```bash
# RoboCasa specifically works with numpy 1.23.x
pip install numpy==1.23.3 --force-reinstall --break-system-packages
```

#### Error: Kitchen generation takes too long

**Solution:**
```bash
# Use faster physics mode during development
python3 robocasa_integration.py --physics-mode fast

# Or disable fixture generation
# Edit robocasa_integration.py and set use_fixtures=False
```

---

## ACADOS MPC Setup

### What is ACADOS?

ACADOS is a fast and embedded-optimization-solver for optimal control and model predictive control. This project uses ACADOS for:

- **Navigation MPC**: Optimal path following for mobile base
- **Torque Control**: Precise arm control with constraints
- **Real-Time Performance**: Fast enough for 100Hz+ control loops

### Installation

#### Ubuntu 22.04 (Complete Guide)

See **[README_UBUNTU_22.04.md](README_UBUNTU_22.04.md)** for complete ACADOS installation guide.

Quick summary:
```bash
# Install dependencies
sudo apt install build-essential cmake git

# Clone ACADOS
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# Build
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4

# Set environment variables
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"
```

#### Verifying ACADOS Installation

```bash
# Check ACADOS is installed
python3 -c "from acados_template import AcadosOcp; print('✅ ACADOS OK')"

# Check environment
echo $ACADOS_SOURCE_DIR  # Should show path to acados

# Generate MPC code
python3 generate_and_compile.py
```

### The `generate_and_compile.py` Script

This script is **crucial** for the project - it generates the optimized C code for MPC:

```bash
python3 generate_and_compile.py
```

**What it does:**

1. **Defines MPC Problem**: Sets up robot dynamics, constraints, cost functions
2. **Generates C Code**: Creates optimized solver code using ACADOS
3. **Compiles**: Builds platform-specific libraries
4. **Creates Interface**: Generates Python wrapper

**Generated Files** (in `c_generated_code/`):

```
c_generated_code/
├── acados_ocp_robot.json       # MPC problem configuration
├── libacados_ocp_solver_robot.so  # Compiled solver (Linux)
├── acados_solver_robot.py      # Python interface
├── acados_sim_robot.py         # Simulation interface
└── [various C files]           # Generated solver code
```

**When to Run:**

| Scenario | Action |
|----------|--------|
| After cloning repository | ✅ Run it |
| After modifying MPC parameters | ✅ Run it |
| Switching to different computer | ✅ Run it |
| Switching OS | ✅ Run it |
| Normal simulation run | ❌ Not needed |

**Why NOT in Git:**

The `c_generated_code/` directory is excluded from git because:
- **Platform-specific**: Compiled for your OS (Linux .so, Windows .dll, macOS .dylib)
- **Large size**: 10-50 MB of compiled code
- **Regeneratable**: Can be created anytime with `generate_and_compile.py`

**Troubleshooting:**

```bash
# If generation fails
# 1. Check ACADOS installation
python3 -c "from acados_template import AcadosOcp; print('OK')"

# 2. Check environment variable
echo $ACADOS_SOURCE_DIR

# 3. Check compiler
gcc --version  # Linux
cl  # Windows

# 4. Clean and regenerate
rm -rf c_generated_code/
python3 generate_and_compile.py
```

---

## Usage

### Basic Usage

```bash
# Run with default settings
python3 main.py

# Run with specific configuration
python3 main.py --config config/robot_config.py
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `w` | Move forward |
| `s` | Move backward |
| `a` | Turn left |
| `d` | Turn right |
| `q` | Quit simulation |
| `r` | Reset simulation |
| `p` | Pause/Resume |
| `Space` | Emergency stop |

### Command Line Arguments

```bash
# Show help
python3 main.py --help

# Use RoboCasa kitchen
python3 main.py --use-robocasa --layout G-shaped --style modern

# Set physics mode
python3 main.py --physics-mode balanced

# Enable LLM control
python3 main.py --llm-control --openai-key YOUR_KEY

# Enable LIDAR mapping
python3 main.py --enable-lidar

# Set simulation speed
python3 main.py --speed 1.5

# Disable visualization (headless)
python3 main.py --headless
```

### Example Workflows

#### 1. Basic Pick-and-Place

```bash
# Run simulation with default settings
python3 main.py

# In Python console or via LLM:
# "Pick up the cup and place it on the table"
```

#### 2. Kitchen Environment Testing

```bash
# Test different kitchen layouts
python3 main.py --use-robocasa --layout G-shaped
python3 main.py --use-robocasa --layout U-shaped
python3 main.py --use-robocasa --layout L-shaped
```

#### 3. Performance Testing

```bash
# Fast physics for development
python3 main.py --physics-mode fast --speed 2.0

# Accurate physics for final testing
python3 main.py --physics-mode accurate --speed 1.0
```

#### 4. LLM-Based Control

```bash
# Set API key
export OPENAI_API_KEY="your-key"

# Run with LLM control
python3 main.py --llm-control

# Give natural language commands:
# "Move to the kitchen counter"
# "Pick up the red mug"
# "Place it on the shelf"
```

### Running Tests

```bash
# Run all tests
pytest tests/

# Run specific test file
pytest tests/test_controllers.py

# Run with coverage
pytest --cov=. tests/

# Run with verbose output
pytest -v tests/
```

---

## Project Structure

```
mujoco_pick_place/
├── main.py                          # Main entry point and system orchestration
├── README.md                        # This file - complete documentation
├── requirements.txt                 # Python dependencies (14 core packages)
├── requirements-dev.txt             # Development dependencies (testing, linting)
├── robocasa_integration.py          # ⭐ RoboCasa kitchen integration (required for kitchen scenes)
├── generate_and_compile.py          # ⭐ ACADOS MPC code generator (creates c_generated_code/)
├── c_generated_code/                # Generated ACADOS solver code (excluded from git)
│                                    # Note: Regenerate with generate_and_compile.py after cloning
├── config/
│   ├── constants.py                 # System constants and parameters
│   ├── robot_config.py              # Robot configuration
│   └── stabilization_config.py      # Stabilization parameters
├── controllers/
│   ├── base_controller.py           # Mobile base control
│   ├── arm_controller.py            # Robotic arm control
│   ├── eso_controller.py            # Extended State Observer
│   ├── dob_controller.py            # Disturbance Observer
│   ├── cascade_controller.py        # ESO-DOB cascade implementation
│   └── mpc_controller.py            # ACADOS MPC navigation controller
├── path_planning/
│   ├── a_star.py                    # A* pathfinding algorithm
│   ├── mpc_hybrid_controller.py     # A* + MPC hybrid navigation
│   └── collision_checker.py         # Collision detection
├── kinematics/
│   ├── forward_kinematics.py        # FK solver
│   ├── inverse_kinematics.py        # IK solver
│   └── jacobian.py                  # Jacobian computation
├── simulation/
│   ├── simulation_manager.py        # Main simulation controller
│   ├── robot.py                     # Robot model wrapper
│   └── sensors.py                   # Sensor implementations (LIDAR, cameras)
├── lidar_mapping/
│   ├── lidar_mapper.py              # LIDAR mapping system
│   └── map_visualizer.py            # Map visualization
├── llm_planner/
│   ├── gpt_planner.py               # GPT-4o integration
│   ├── command_parser.py            # Natural language parsing
│   └── task_executor.py             # High-level task execution
├── tasks/
│   ├── pick_and_place.py            # Pick-and-place task
│   ├── navigation.py                # Navigation task
│   └── multi_robot.py               # Multi-robot coordination
├── model/
│   └── stanford_tidybot/
│       ├── scene.xml                # Main scene file
│       ├── scene_robocasa.xml       # RoboCasa integration scene
│       ├── tidybot.xml              # Robot1 model
│       └── tidybot_robot2.xml       # Robot2 model
└── scripts/
    ├── installation/
    │   ├── fix_robocasa_deps.py             # RoboCasa dependency checker
    │   └── install_robocasa_complete.sh     # Automated RoboCasa installer
    └── [various utility scripts]
```

### Key Files Explained

#### Core Files

- **main.py**: Entry point that initializes the simulation system
- **robocasa_integration.py** ⭐: Required for RoboCasa kitchen environments
- **generate_and_compile.py** ⭐: Generates ACADOS MPC solver code

#### Configuration

- **config/robot_config.py**: Robot parameters (DOF, limits, gains)
- **config/constants.py**: System-wide constants
- **config/stabilization_config.py**: Control tuning parameters

#### Controllers

- **mpc_controller.py**: ACADOS-based navigation MPC
- **cascade_controller.py**: ESO-DOB cascade for arm control
- **eso_controller.py**: Extended State Observer for disturbance estimation
- **dob_controller.py**: Disturbance Observer for rejection

#### Dependencies

- **requirements.txt**: 14 core packages with version constraints
- **requirements-dev.txt**: Development tools (pytest, black, flake8, mypy)

---

## Development

### ACADOS MPC Code Generation

Detailed in the [ACADOS MPC Setup](#acados-mpc-setup) section above.

### Code Quality

```bash
# Format code
black .
isort .

# Lint code
flake8 .
pylint **/*.py

# Type checking
mypy .

# Run all checks
black . && isort . && flake8 . && mypy .
```

### Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=.

# Run specific test
pytest tests/test_mpc.py

# Run with verbose output
pytest -v
```

### Debugging

```bash
# Run with Python debugger
python3 -m pdb main.py

# Enable debug logging
export LOG_LEVEL=DEBUG
python3 main.py

# Visual debugging with MuJoCo viewer
python3 main.py --debug-render
```

### Profiling

```bash
# Profile with cProfile
python3 -m cProfile -o profile.stats main.py

# Analyze results
python3 -m pstats profile.stats

# Memory profiling
python3 -m memory_profiler main.py
```

---

## Troubleshooting

### Common Issues

#### 1. Import Errors

**Problem**: `ModuleNotFoundError: No module named 'X'`

**Solutions:**
```bash
# Check Python version
python3 --version  # Should be 3.8+

# Reinstall dependencies
pip install -r requirements.txt --force-reinstall

# Check pip is using correct Python
python3 -m pip --version

# Use virtual environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

#### 2. MuJoCo Errors

**Problem**: `ImportError: libmujoco.so: cannot open shared object file`

**Solutions:**
```bash
# Install MuJoCo
pip install mujoco --upgrade

# For headless systems
export MUJOCO_GL=osmesa

# Check installation
python3 -c "import mujoco; print(mujoco.__version__)"
```

#### 3. NumPy Version Conflicts

**Problem**: `AttributeError: module 'numpy' has no attribute 'X'`

**Solutions:**
```bash
# Use compatible version
pip install "numpy>=1.21.0,<2.0.0" --force-reinstall

# For RoboCasa specifically
pip install numpy==1.23.3 --force-reinstall
```

#### 4. ACADOS MPC Errors

**Problem**: `ModuleNotFoundError: No module named 'acados_template'`

**Solutions:**
```bash
# Check ACADOS installation
echo $ACADOS_SOURCE_DIR

# Reinstall ACADOS Python interface
cd $ACADOS_SOURCE_DIR/interfaces/acados_template
pip install -e .

# Regenerate MPC code
python3 generate_and_compile.py
```

#### 5. RoboCasa Installation Issues

See detailed solutions in [RoboCasa Troubleshooting](#robocasa-troubleshooting) section above.

#### 6. Performance Issues

**Problem**: Simulation running slowly

**Solutions:**
```bash
# Use fast physics mode
python3 main.py --physics-mode fast

# Reduce rendering quality
python3 main.py --render-quality low

# Disable visualization
python3 main.py --headless

# Check system resources
htop  # Linux
```

#### 7. GPU/OpenGL Issues

**Problem**: `ERROR: GLEW initalization error: Missing GL version`

**Solutions:**
```bash
# Use CPU rendering (headless systems)
export MUJOCO_GL=osmesa

# Update GPU drivers
# NVIDIA: https://www.nvidia.com/download/index.aspx
# AMD: https://www.amd.com/en/support

# Check OpenGL support
glxinfo | grep "OpenGL version"
```

### Getting Help

If you encounter issues not covered here:

1. **Check Documentation**:
   - This README.md
   - INSTALL_GUIDE.md
   - README_UBUNTU_22.04.md

2. **Check Dependencies**:
   ```bash
   pip list | grep -E "numpy|mujoco|opencv|scipy"
   ```

3. **Check Environment**:
   ```bash
   python3 --version
   pip --version
   echo $ACADOS_SOURCE_DIR
   ```

4. **Enable Debug Logging**:
   ```bash
   export LOG_LEVEL=DEBUG
   python3 main.py
   ```

5. **Create GitHub Issue**:
   - Include error messages
   - Include system information
   - Include steps to reproduce

---

## Additional Resources

### Project Documentation
- **Installation Guide**: This README (comprehensive)
- **Quick Start (Ubuntu 22.04)**: See README_UBUNTU_22.04.md
- **Development Tools**: See requirements-dev.txt

### External Documentation
- **[MuJoCo Documentation](https://mujoco.readthedocs.io/)**
- **[ACADOS Documentation](https://docs.acados.org/)**
- **[RoboCasa Documentation](https://robocasa.ai/docs)**
- **[RoboSuite Documentation](https://robosuite.ai/docs)**
- **[CasADi Documentation](https://web.casadi.org/docs/)**

### Research Papers
- TidyBot: Personalized Robot Assistance with Large Language Models
- ACADOS: A Modular Open-Source Framework for Fast Embedded Optimal Control
- Extended State Observer Based Control for Systems with Mismatched Uncertainties

### Related Projects
- **Stanford TidyBot**: https://tidybot.cs.stanford.edu/
- **RoboCasa**: https://robocasa.ai/
- **RoboSuite**: https://robosuite.ai/
- **ACADOS**: https://github.com/acados/acados

---

## Credits

### Authors
- **Hoyon Na** - Lead Developer

### Acknowledgments
- **Stanford TidyBot Team** - Robot model and concept
- **RoboCasa Team** - Kitchen environments
- **RoboSuite Team** - Simulation framework
- **ACADOS Team** - MPC solver
- **MuJoCo Team** - Physics engine

### License
This project is licensed under the MIT License - see the LICENSE file for details.

### Citation
If you use this project in your research, please cite:
```bibtex
@software{mujoco_dual_robot_2024,
  author = {Na, Hoyon},
  title = {MuJoCo Dual-Robot Navigation and Pick-and-Place System},
  year = {2024},
  url = {https://github.com/HoyongNa/mujoco_pick_place}
}
```

---

## Support

### Community
- GitHub Issues: Report bugs or request features
- GitHub Discussions: Ask questions, share ideas
- Pull Requests: Contributions welcome!

### Contact
- GitHub: [@HoyongNa](https://github.com/HoyongNa)
- Email: [Contact through GitHub]

---

**Last Updated**: November 2024  
**Version**: 4.0  
**Status**: Active Development

---

## Quick Links

- [Installation](#installation) - Get started
- [RoboCasa Setup](#robocasa-kitchen-environments) - Kitchen environments
- [Usage](#usage) - Run simulations
- [Troubleshooting](#troubleshooting) - Fix common issues
- [Project Structure](#project-structure) - Understand the codebase

---

**Ready to start? Run:**
```bash
pip install -r requirements.txt
python3 generate_and_compile.py
python3 main.py
```

Enjoy! 🤖🎉
