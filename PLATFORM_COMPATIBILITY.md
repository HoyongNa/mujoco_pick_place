# Platform Compatibility Guide

## Overview
This project has been updated to use `pynput` instead of `keyboard` library for better cross-platform compatibility, especially for macOS users.

## Changes Made

### 1. Dependency Update
- **Before**: `keyboard>=0.13.5` (Windows/Linux only)
- **After**: `pynput>=1.7.0` (Windows/macOS/Linux)

### 2. Modified Files
- `requirements.txt` - Updated dependency
- `controllers/base/keyboard_handler.py` - Rewritten using pynput
- `controllers/base/mobility_controller.py` - Added cleanup for pynput listener
- `main.py` - Updated to use pynput for function keys

## Installation

### All Platforms
```bash
pip install -r requirements.txt
```

### Platform-Specific Notes

#### macOS
- No additional permissions needed for basic keyboard monitoring
- If you encounter permission issues, grant Terminal/IDE accessibility permissions in System Preferences

#### Linux
- May require running with sudo for global keyboard access:
  ```bash
  sudo python main.py
  ```
- Or add user to `input` group:
  ```bash
  sudo usermod -a -G input $USER
  ```

#### Windows
- Works out of the box
- No special permissions required

## Key Mapping

The following keys are used in the simulation:

### Navigation (Numpad)
- `8` - Forward
- `5` - Backward
- `4` - Left
- `6` - Right
- `7` - Rotate left
- `9` - Rotate right
- `2` - Stop

### Room Navigation
- `F8` - Navigate to Room 1 (Northwest)
- `F9` - Navigate to Room 2 (Northeast)
- `F10` - Navigate to Room 3 (Southwest)
- `F11` - Navigate to Room 4 (Southeast)

### Actions
- `Space` - Execute Pick & Place task
- `ESC` - Exit simulation

## Troubleshooting

### Issue: Keys not responding on macOS
**Solution**: Grant accessibility permissions to your terminal or IDE:
1. System Preferences → Security & Privacy → Privacy
2. Select Accessibility
3. Add your Terminal/IDE application

### Issue: Permission denied on Linux
**Solution**: Run with sudo or add user to input group (see Linux section above)

### Issue: Numpad keys not working
**Solution**: Ensure Num Lock is enabled. The implementation handles both numpad and regular number keys.

## Technical Details

### pynput Implementation
The new implementation uses:
- `pynput.keyboard.Listener` for event-based key monitoring
- Thread-safe key state tracking with `threading.Lock`
- Proper cleanup with `listener.stop()` method
- Cross-platform virtual key code handling

### Key Differences from keyboard library
1. **Event-based**: pynput uses callbacks instead of polling
2. **Thread-safe**: Built-in thread safety for multi-threaded applications
3. **Better macOS support**: Native support without kernel extensions
4. **Resource cleanup**: Explicit listener lifecycle management

## Testing

To verify the keyboard input is working correctly:

```python
# Test script
from pynput import keyboard

def on_press(key):
    try:
        print(f'Key pressed: {key.char}')
    except AttributeError:
        print(f'Special key pressed: {key}')

def on_release(key):
    if key == keyboard.Key.esc:
        return False

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
```

## Migration from keyboard to pynput

If you have custom code using the `keyboard` library, here's a migration guide:

### Before (keyboard)
```python
import keyboard

if keyboard.is_pressed('space'):
    do_something()
```

### After (pynput)
```python
from pynput import keyboard
import threading

pressed_keys = set()
lock = threading.Lock()

def on_press(key):
    if key == keyboard.Key.space:
        with lock:
            pressed_keys.add('space')

def on_release(key):
    if key == keyboard.Key.space:
        with lock:
            pressed_keys.discard('space')

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Check if pressed
with lock:
    if 'space' in pressed_keys:
        do_something()
```

## Performance Considerations

- pynput uses minimal CPU resources with event-based monitoring
- Thread-safe implementation adds minimal overhead
- Proper cleanup prevents resource leaks
- No polling means better battery life on laptops

## Contributors

This cross-platform compatibility update was implemented to ensure the project works seamlessly across Windows, macOS, and Linux platforms.
