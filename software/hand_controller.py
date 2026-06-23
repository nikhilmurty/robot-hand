"""Hand Controller Class
Handles control of 5 dynamixel motors that control the fingers on a tension driven robotic hand
"""
from dynamixel_sdk import *
import time

class HandController:
    ADDR_TORQUE_ENABLE          = 64
    ADDR_TARGET_POSITION          = 116
    ADDR_ACTUAL_POSITION       = 0

    PROTOCOL_VERSION            = 2.0
    DXL_ID                      = 1
    BAUDRATE                    = 57600

    THUMB                       = 1
    POINTER                     = 2
    MIDDLE                      = 3
    RING                        = 4
    PINKY                       = 5


    def __init__(self, DEVICENAME ='COM7'):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to open port or set baudrate. Check your DEVICENAME!")
            quit()

    def close_port(self):
        self.packetHandler.write1ByteTxRx(self.portHandler, 1, self.ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()

    def enable(self, motor):
        self.packetHandler.write1ByteTxRx(self.portHandler, motor, self.ADDR_TORQUE_ENABLE, 1)
        print(f"Torque enabled on joint #{motor}")

    def disable(self, motor):
        self.packetHandler.write1ByteTxRx(self.portHandler, motor, self.ADDR_TORQUE_ENABLE, 0)
        print(f"Torque disabled on joint #{motor}")

    def setPosition(self, motor, position):
        """transmits position data to the motor

        Args:
            motor (int): motor number in the chain
            position (int): position of the motor in degrees
        """
        self.packetHandler.write4ByteTxRx(self.portHandler, motor, self.ADDR_TARGET_POSITION, position)

    def setHandPosition(self,positions):
        """Sets the positions of each of the fingers

        Args:
            positions (array of ints): an array containing all the joints starting from thumb (motor enumeration starts at 1)
        """
        for i, position in enumerate(positions):
            angle = self.finger2motor(position)
            self.setPosition(i+1, position)

    def finger2motor(self, finger_angle):
        """Converts finger angle from finger capture to motor angle

        Args:
            finger_angle (_type_): angle from the finger_capture code
        """
        #TODO Refine all of this
        
        min_angle, max_angle = 15, 90.0
        min_motor, max_motor = 3000, 4000
        
        # Clamp input to safe bounds
        clamped_angle = max(min_angle, min(finger_angle, max_angle))
        
        # Map from [0, 90] -> [1000, 2000]
        motor_position = min_motor + (clamped_angle - min_angle) * (max_motor - min_motor) / (max_angle - min_angle)
        return int(motor_position)

if __name__ == "__main__":
    motor = HandController()
    i = 0
    motor.enable(1)
    for i in range(1,4095,100):
        motor.setPosition(1,i)
        time.sleep(0.01)

    for i in range(4095,1,100):
        motor.setPosition(1,i)
        time.sleep(0.01)
    motor.disable(1)