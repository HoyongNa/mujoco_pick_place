#!/bin/bash

# RoboCasa Complete Fix Script
# This installs all missing dependencies

echo "=============================================="
echo "RoboCasa Complete Installation Fix"
echo "=============================================="
echo ""
echo "This script will:"
echo "  1. Install robosuite (missing critical dependency)"
echo "  2. Install robocasa from local directory"
echo "  3. Verify the installation"
echo ""

# Install robosuite first (CRITICAL - missing from setup.py!)
echo "Step 1: Installing robosuite..."
pip install robosuite --break-system-packages

if [ $? -ne 0 ]; then
    echo "❌ Failed to install robosuite"
    exit 1
fi

echo ""
echo "Step 2: Installing robocasa..."
cd ~/mujoco_pick_place/robocasa
pip install -e . --break-system-packages

if [ $? -ne 0 ]; then
    echo "❌ Failed to install robocasa"
    exit 1
fi

echo ""
echo "Step 3: Verifying installation..."
python3 << 'EOF'
try:
    import robosuite
    print("✅ robosuite imported successfully")
except Exception as e:
    print(f"❌ robosuite import failed: {e}")

try:
    import robocasa
    print("✅ robocasa imported successfully")
except Exception as e:
    print(f"❌ robocasa import failed: {e}")

try:
    from robocasa.environments.kitchen.kitchen import Kitchen
    print("✅ Kitchen class imported successfully")
    print("")
    print("="*50)
    print("✅ ALL CHECKS PASSED!")
    print("="*50)
except Exception as e:
    print(f"❌ Kitchen class import failed: {e}")
    print("")
    print("Please check the error messages above")
EOF

echo ""
echo "Installation complete!"
echo ""
echo "Next steps:"
echo "  cd ~/mujoco_pick_place"
echo "  python3 main.py"
