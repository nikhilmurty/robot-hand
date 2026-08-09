# Robot Hand

This repository contains the design, CAD models, and control software for a 5-DOF tension-driven robotic hand controlled via live webcam hand tracking.

## Quick Start Overview

To run this project, you will need to:
1. **Connect & Configure Hardware**:
   - Connect **5x Dynamixel XL-330 motors** to an **OpenRB-150** controller board (flashed with USB Passthrough / `usb_to_dxl` firmware).
   - Use **ROBOTIS DYNAMIXEL Wizard 2.0** to set baud rate to `57600` and assign motor IDs `1` (Pinky), `2` (Ring), `3` (Middle), `4` (Pointer), `5` (Thumb).
   - For detailed assembly instructions, see [hardware/README.md](hardware/README.md).

2. **Configure & Run Software**:
   - Install Python dependencies (`uv sync` or `pip install -r ...`).
   - Verify/update the serial port (`DEVICENAME`) in `software/hand_controller.py` (e.g. `COM7`, `/dev/ttyACM0`, or `/dev/tty.usbmodem*`).
   - Run the main capture script:
     ```bash
     uv run python software/finger_capture.py
     ```
   - For detailed setup, auto-tensioning overview, and software architecture, see [software/README.md](software/README.md).

## Repository Structure

- **[hardware/](hardware/README.md)**: 3D CAD files (STEP, 3MF), bill of materials, Onshape link, and hardware assembly guide. Licensed under CERN OHL v2 Permissive.
- **[software/](software/README.md)**: Python scripts for MediaPipe hand tracking (`finger_capture.py`) and Dynamixel servo control (`hand_controller.py`).
 