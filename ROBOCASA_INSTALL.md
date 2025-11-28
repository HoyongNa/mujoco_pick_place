# RoboCasa Installation Quick Reference

## Problem
```
ModuleNotFoundError: No module named 'robocasa.environments'
```

## Root Cause
RoboCasa's `setup.py` is missing `robosuite` as a dependency. You must install it manually.

## Quick Fix

```bash
# Step 1: Install robosuite (CRITICAL - missing dependency!)
pip install robosuite --break-system-packages

# Step 2: Install robocasa
cd ~/mujoco_pick_place/robocasa
pip install -e . --break-system-packages

# Step 3: Verify installation
python3 -c "from robocasa.environments.kitchen.kitchen import Kitchen; print('✅ Success!')"
```

## Automated Solutions

### Option 1: Diagnostic Tool (Recommended)
```bash
cd ~/mujoco_pick_place
python3 fix_robocasa_deps.py
```

This interactive tool will:
- Check all dependencies
- Show what's missing
- Offer to install missing packages
- Provide detailed diagnostics

### Option 2: Automated Script
```bash
cd ~/mujoco_pick_place
bash install_robocasa_complete.sh
```

This script will:
- Install robosuite
- Install robocasa
- Verify both packages
- Test Kitchen class import

## Verification Checklist

Run each command and make sure you see ✅:

```bash
python3 -c "import robosuite; print('✅')"
python3 -c "import robocasa; print('✅')"
python3 -c "from robocasa.environments.kitchen.kitchen import Kitchen; print('✅')"
python3 -c "from robocasa.models.scenes.kitchen_arena import KitchenArena; print('✅')"
```

## Common Warnings (Safe to Ignore)

These warnings are normal and can be ignored:
- "No private macro file found"
- "Could not import robosuite_models"
- "Could not load the mink-based whole-body IK"
- "mimicgen environments not imported"

## Troubleshooting

### Still Getting Import Errors?

1. Check Python version:
   ```bash
   python3 --version  # Should be 3.8+
   ```

2. Check which Python pip is using:
   ```bash
   python3 -m pip --version
   ```

3. Restart your terminal after installation

4. Try reinstalling:
   ```bash
   pip uninstall robocasa robosuite
   pip install robosuite --break-system-packages
   cd ~/mujoco_pick_place/robocasa && pip install -e . --break-system-packages
   ```

## After Successful Installation

Run your simulation:
```bash
cd ~/mujoco_pick_place
python3 main.py
```

Expected output:
```
[SimulationManager] Using RoboCasa kitchen: G-shaped - modern
[SimulationManager] ⏳ Generating kitchen fixtures... (10-30 seconds)
INFO: Creating RoboCasa kitchen: layout=G-shaped (id=7), style=modern (id=3)
INFO: ✓ Kitchen created with 450+ bodies
INFO: 🎉 FULL KITCHEN with fixtures!
```

## Need More Help?

See the complete documentation:
- README.md - Main project documentation
- README_UBUNTU_22.04.md - Ubuntu-specific installation guide
- COMPLETE_SOLUTION.txt - Detailed troubleshooting (in helper files)
