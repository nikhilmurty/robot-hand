# Hardware

This folder contains all the 3D design and print files for the robot hand and fingers. View the full 3D assembly on [Onshape](https://cad.onshape.com/documents/e974f0bc1062bd7b6ade647d/w/e4fd7f55056d36383327a202/e/fd9bded8c1b01162ac553002?renderMode=0&uiState=6a4c9dbf046b862a1ab126a8).

## Parts & Bill of Materials

To build the hardware setup, you will need:

- **5x Dynamixel XL-330 motors**
- **1x ROBOTIS OpenRB-150 controller board** (serves as the USB-to-Dynamixel communication bridge)
- **1x USB-C Cable** (to connect OpenRB-150 to your host PC)
- **1x 5V Power Supply / Terminal block** (to power the Dynamixel motors via the OpenRB-150 board)
- **1x USB Camera / Webcam** (for hand tracking)
- **30x Bearings** (5mm ID, 8mm OD, e.g. MR85-2RS)
- **15x Dowel Pins** (M5 x 14mm)
- **Durable Cable / String** (e.g. 40 lbs braided fishing line for tendon control)
- **5x Springs** (0.5 x 5.5 x 35.5 mm for finger return tensioning)
- **Assorted M2 and M3 Screws and nuts** (I bought sets of these on amazon)
- **M2 and M3 Tap** (M2 for tapping holes in the dynamixels and M3 for the hand)

---

## Hardware Configuration & Wiring

### 1. OpenRB-150 Controller Board
- Connect the **OpenRB-150** to your computer via USB-C.
- Ensure the OpenRB-150 is flashed with **USB Passthrough firmware** (such as `usb_to_dxl` from the OpenRB-150 / DYNAMIXEL library in Arduino IDE) so it functions as a serial bridge between your computer and the servos.

### 2. Dynamixel Motor ID & Baud Rate Configuration
Use **ROBOTIS DYNAMIXEL Wizard 2.0** to assign IDs and set the baud rate before assembling:
- **Baud Rate**: `57600` (all motors must match)
- **Motor IDs**:
  - `ID 1`: Pinky Finger
  - `ID 2`: Ring Finger
  - `ID 3`: Middle Finger
  - `ID 4`: Pointer (Index) Finger
  - `ID 5`: Thumb

### 3. Assembly & Wiring
- Daisy-chain or plug all 5 Dynamixel XL-330 motors into the TTL 3-pin ports on the OpenRB-150.
- Mount the motors to the spools in the main palm assembly.
- Route string through finger joint channels and secure to tensioning springs as detailed in the Onshape model.


