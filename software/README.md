# Software

This directory contains the Python software to control the 5-DOF tension-driven robotic hand using live webcam hand landmark tracking.

## Prerequisites

### 1. Hardware Requirements
- **Webcam**: Standard USB camera connected to your host PC.
- **OpenRB-150 Controller Board**: Connected via USB and configured with USB passthrough firmware (`usb_to_dxl`).
- **5x Dynamixel XL-330 Motors**: Connected to the OpenRB-150 board and configured as follows:
  - **Baudrate**: `57600`
  - **IDs**: 1 (Pinky), 2 (Ring), 3 (Middle), 4 (Pointer), 5 (Thumb)

### 2. Software Requirements
- **Python**: Version `3.12+`
- **Dependencies**:
  - `dynamixel-sdk`
  - `mediapipe`
  - `opencv-python`
  - `numpy`

---

## Installation & Setup

1. **Install Dependencies** using `uv` (recommended) or `pip`:
   ```bash
   uv sync
   ```
   *Alternatively, using standard `pip`:*
   ```bash
   pip install dynamixel-sdk mediapipe opencv-python numpy
   ```

2. **Identify Your Serial Port**:
   Check which port the OpenRB-150 is mounted on:
   - **Windows**: `COM3`, `COM7`, etc.
   - **macOS**: `/dev/tty.usbmodem*` or `/dev/cu.usbmodem*`
   - **Linux**: `/dev/ttyACM0` or `/dev/ttyUSB0`

3. **Configure the Serial Port**:
   In `software/hand_controller.py`, ensure the default `DEVICENAME` matches your system's serial port:
   ```python
   # Example in software/hand_controller.py:
   def __init__(self, DEVICENAME='COM7'): # Change 'COM7' to your device port (e.g. '/dev/ttyACM0')
   ```

---

## How to Run

Run `finger_capture.py` to start the hand tracking and control system:

```bash
uv run python software/finger_capture.py
```
*(or `python software/finger_capture.py` if using standard Python virtual environment)*

---

## What Happens When Running `finger_capture.py`

1. **Camera & Landmarker Setup**: Initializes OpenCV video capture from webcam (Device 0) and loads the MediaPipe `hand_landmarker.task` model.
2. **Auto-Tensioning Sequence**: 
   - `HandController` initializes the OpenRB-150 connection.
   - Executes `auto_tension()` across all 5 Dynamixels (IDs 1 through 5).
   - Servos pull tension until current limits are detected to establish the open/closed position bounds for each finger.
3. **Live Hand Tracking & Motor Control**:
   - A thread reads frames from the webcam and runs MediaPipe hand landmark detection.
   - A second thread extracts 3D joint vectors for each finger (thumb, pointer, middle, ring, pinky) to compute flexion angles.
   - Angles are transformed into motor target positions and synced over the serial port to the OpenRB-150 board.
4. **Display & Exit**:
   - Displays a live video feed window with skeleton landmark overlays.
   - Press **`q`** while focused on the video window to quit, disable motor torque, and safely close the serial connection.
