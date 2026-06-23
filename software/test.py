import time
from dynamixel_sdk import *

# --- CONFIGURATION ---
# XL330 Control Table Addresses (Protocol 2.0)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 0

PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1           # Default Dynamixel ID
BAUDRATE                    = 57600       # Default XL330 baudrate
DEVICENAME                  = 'COM7'      # Windows: 'COMX' | Linux/Mac: '/dev/ttyACM0'

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open Port & Set Baudrate
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("Failed to open port or set baudrate. Check your DEVICENAME!")
    quit()

# 1. Enable Torque (1 = Enable, 0 = Disable)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
print("Torque enabled.")

# 2. Move to Position A (e.g., 1000 out of 0-4095)
print("Moving to Position A...")
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 1000)
time.sleep(2)

# 3. Move to Position B (e.g., 2000)
print("Moving to Position B...")
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 2000)
time.sleep(2)

# 4. Clean up: Disable torque and close port
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
portHandler.closePort()
print("Torque disabled and port closed.")