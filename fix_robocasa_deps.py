#!/usr/bin/env python3
"""
RoboCasa Dependency Diagnostic and Fix Script
Checks all dependencies and installs missing ones
"""

import sys
import subprocess
import importlib.util

def check_module(module_name):
    """Check if a module is installed"""
    spec = importlib.util.find_spec(module_name)
    return spec is not None

def install_package(package_name, flags=None):
    """Install a package using pip"""
    cmd = [sys.executable, "-m", "pip", "install", package_name]
    if flags:
        cmd.extend(flags)
    
    print(f"Installing {package_name}...")
    try:
        subprocess.run(cmd, check=True)
        return True
    except subprocess.CalledProcessError:
        return False

def main():
    print("="*70)
    print("RoboCasa Dependency Diagnostic")
    print("="*70)
    print()
    
    # Check dependencies
    dependencies = {
        "numpy": "numpy>=1.23.0",
        "mujoco": "mujoco==3.2.6",
        "robosuite": "robosuite",  # CRITICAL: Missing from setup.py!
        "robocasa": None,  # Check if robocasa itself is installed
    }
    
    missing = []
    installed = []
    
    print("Checking dependencies...")
    print()
    
    for module, package in dependencies.items():
        if check_module(module):
            print(f"  ✅ {module:<15} - installed")
            installed.append(module)
        else:
            print(f"  ❌ {module:<15} - NOT INSTALLED")
            if package:
                missing.append(package)
    
    print()
    print("="*70)
    
    # Special diagnostic for robocasa
    if "robocasa" not in installed:
        print("\n❌ RoboCasa is not installed!")
        print("\nTo install RoboCasa:")
        print("  cd ~/mujoco_pick_place/robocasa")
        print("  pip install -e . --break-system-packages")
        print()
    else:
        print("\n✅ RoboCasa package is installed")
        
        # Try to import specific modules
        print("\nTesting RoboCasa modules:")
        
        try:
            import robocasa
            print(f"  ✅ robocasa - OK")
            print(f"     Location: {robocasa.__file__}")
        except Exception as e:
            print(f"  ❌ robocasa - FAILED: {e}")
        
        try:
            from robocasa.environments.kitchen.kitchen import Kitchen
            print(f"  ✅ Kitchen class - OK")
        except Exception as e:
            print(f"  ❌ Kitchen class - FAILED: {e}")
            print(f"     This usually means robosuite is not installed!")
    
    # Check for robosuite separately
    print()
    print("="*70)
    print("ROBOSUITE CHECK (Critical Dependency)")
    print("="*70)
    
    if "robosuite" not in installed:
        print("\n❌ ROBOSUITE IS NOT INSTALLED!")
        print("\n⚠️  RoboCasa REQUIRES robosuite but it's not in setup.py!")
        print("   You need to install it manually.")
        print()
        
        response = input("Install robosuite now? (y/n): ")
        if response.lower() in ['y', 'yes']:
            print("\nInstalling robosuite...")
            if install_package("robosuite", ["--break-system-packages"]):
                print("✅ robosuite installed successfully!")
                print("\nNow reinstalling robocasa to ensure compatibility...")
                
                # Reinstall robocasa
                import os
                robocasa_path = os.path.expanduser("~/mujoco_pick_place/robocasa")
                if os.path.exists(robocasa_path):
                    subprocess.run([
                        sys.executable, "-m", "pip", "install", 
                        "-e", robocasa_path, 
                        "--break-system-packages"
                    ])
                
                print("\n" + "="*70)
                print("✅ Installation complete!")
                print("="*70)
                print("\nTest the installation:")
                print("  python3 -c 'from robocasa.environments.kitchen.kitchen import Kitchen; print(\"✅ Success!\")'")
            else:
                print("❌ Failed to install robosuite")
    else:
        print("\n✅ robosuite is installed")
    
    print()
    print("="*70)
    print("SUMMARY")
    print("="*70)
    
    if missing:
        print(f"\n⚠️  {len(missing)} dependencies missing:")
        for pkg in missing:
            print(f"  - {pkg}")
        print("\nInstall missing dependencies:")
        print(f"  pip install {' '.join(missing)} --break-system-packages")
    else:
        print("\n✅ All dependencies are installed!")
        print("\nYou should now be able to run:")
        print("  cd ~/mujoco_pick_place")
        print("  python3 main.py")

if __name__ == "__main__":
    main()
