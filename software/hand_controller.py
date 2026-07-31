"""Hand Controller Class
Handles control of 5 dynamixel motors that control the fingers on a tension driven robotic hand
"""
from dynamixel_sdk import *
import time
import threading

class Finger:
    def __init__(self, name, motor_id, min = 0, max = 120):
        self.name = name
        self.motor_id = motor_id
        self.open_pos = 0
        self.closed_pos = 0

        self.min_angle = min
        self.max_angle = max

        self.actual_pos = 0.0
        self.actual_current = 0.0
        self.actual_velocity = 0.0

        self.target_pos = 0


    def calculate_motor_angle(self, finger_angle):
        # Clamp input to safe bounds
        clamped_angle = max(self.min_angle, min(finger_angle, self.max_angle))
        
        # Map from [min, max] -> [closed, open]
        self.target_pos = self.open_pos + (clamped_angle - self.min_angle) * (self.closed_pos - self.open_pos) / (self.max_angle - self.min_angle)
        return int(self.target_pos)
    
    def update_values(self, pos, current, velocity):
        self.actual_pos = pos
        self.actual_current = current
        self.actual_velocity = velocity


class HandController:
    ADDR_TORQUE_ENABLE          = 64
    ADDR_OPERATING_MODE         = 11          
    ADDR_TARGET_POSITION        = 116
    ADDR_GOAL_CURRENT           = 102
    LEN_GOAL_POSITION           = 4

    ADDR_ACTUAL_POSITION        = 132
    ADDR_ACTUAL_CURRENT         = 126
    ADDR_ACTUAL_VELOCITY        = 128

    CURRENT_POSITION_OPERATION  = 5         #Current based position
    MAX_CURRENT                 = 300       #current cap until backdrivable
    TENSION_CURRENT             = 20

    PROTOCOL_VERSION            = 2.0
    DXL_ID                      = 1
    BAUDRATE                    = 57600

    THUMB                       = 5
    POINTER                     = 4
    MIDDLE                      = 3
    RING                        = 2
    PINKY                       = 1
    MOTOR_IDS                   = [1,2,3,4,5]


    def __init__(self, DEVICENAME ='COM7'):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to open port or set baudrate. Check your DEVICENAME!")
            quit()

        #create the hand
        self.hand = {
            "THUMB"   : Finger("THUMB", self.THUMB, 15, 75),
            "POINTER" : Finger("POINTER", self.POINTER, 0, 120),
            "MIDDLE"  : Finger("MIDDLE", self.MIDDLE, 0, 120),
            "RING"    : Finger("RING", self.RING, 0, 120),
            "PINKY"   : Finger("PINKY", self.PINKY, 0, 120)
        }

        self.auto_tension()
        print("Finished tensioning")

        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_TARGET_POSITION, 4)
        self.groupSyncRead  = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_ACTUAL_CURRENT, 10)

        for finger in self.hand.values():
            self.groupSyncRead.addParam(finger.motor_id)

        self.hand_thread = threading.Thread(target=self.run, daemon=True)

    def close_port(self):
        for finger in self.hand.values():
            self.disable(finger.motor_id)

        self.portHandler.closePort()

    def enable(self, motor):
        self.packetHandler.write1ByteTxRx(self.portHandler, motor, self.ADDR_TORQUE_ENABLE, 1)
        print(f"Torque enabled on joint #{motor}")

    def disable(self, motor):
        self.packetHandler.write1ByteTxRx(self.portHandler, motor, self.ADDR_TORQUE_ENABLE, 0)
        print(f"Torque disabled on joint #{motor}")

    def read_hand(self):
        """Reads values coming from dynamixels, saves to the motor classes
        """

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Sync read failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return

        for finger in self.hand.values():
            motor = finger.motor_id

            if self.groupSyncRead.isAvailable(motor, self.ADDR_ACTUAL_POSITION, 4):
                # Retrieve the 4-byte position data
                curr = self.groupSyncRead.getData(motor, self.ADDR_ACTUAL_CURRENT, 2)
                if curr > 32767:
                    curr -= 65536

                vel = self.groupSyncRead.getData(motor, self.ADDR_ACTUAL_VELOCITY, 4)
                if vel > 2147483647:
                    vel -= 4294967296

                pos = self.groupSyncRead.getData(motor, self.ADDR_ACTUAL_POSITION, 4)
                # Convert from unsigned to signed 32-bit integer
                if pos > 2147483647:
                    pos -= 4294967296
                
                finger.update_values(pos, curr, vel)
            else:
                print(f"[ID:{motor}] groupSyncRead data not available")

    def write_hand(self):
        """writes values to hands
        """

        self.groupSyncWrite.clearParam()

        for finger in self.hand.values():
            motor  = finger.motor_id
            target = int(finger.target_pos)

            dxl_addparam_result = self.groupSyncWrite.addParam(motor, target.to_bytes(4, 'little', signed=True))

            if not dxl_addparam_result:
                print(f"addParam failed for joint #{motor}")
                return
        

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Sync write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return

    def start(self):
        self.is_running = True
        self.hand_thread.start()

    def stop(self):
        self.is_running = False
        self.hand_thread.join(timeout=2.0)
    def run(self):
        try:
            while self.is_running:
                self.read_hand()
                self.write_hand()

                time.sleep(0.01)

        except Exception as e:
            print(f"Error in main loop: {e}")

        finally:
            self.close_port()
                



    def auto_tension(self):
        """Auto tensions every finger and gets max and min positions
        """

        for name,finger in self.hand.items():
            motor = finger.motor_id

            tensioned = False
            maxxed = False
            
            #set operating mode to current based position
            self.packetHandler.write1ByteTxRx(self.portHandler, motor , self.ADDR_OPERATING_MODE, self.CURRENT_POSITION_OPERATION)

            #Start Motor
            self.enable(motor)

            #set goal current to tension and move until tensioned
            self.packetHandler.write2ByteTxRx(self.portHandler, motor , self.ADDR_GOAL_CURRENT, self.TENSION_CURRENT)

            start_pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_ACTUAL_POSITION)
            if start_pos > 2147483647:
                start_pos -= 4294967296
            tension_pos = start_pos + 3000
            self.packetHandler.write4ByteTxRx(self.portHandler, motor , self.ADDR_TARGET_POSITION, tension_pos)

            #read position and current until it doesnt change
            while not tensioned:
                pos, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_ACTUAL_POSITION)
                if pos > 2147483647:
                    pos -= 4294967296
                curr, _, _ =  self.packetHandler.read2ByteTxRx(self.portHandler, motor, self.ADDR_ACTUAL_CURRENT)

                if curr > 32767:
                    curr -= 65536
                curr = abs(curr)

                if curr >= self.TENSION_CURRENT:
                    finger.open_pos = pos           #set open pos to tensioned value
                    tensioned = True
                
                time.sleep(0.1)
            
            #To get max position, we just need to set the current to the max current threshold
            prev_pos = 0
            self.packetHandler.write2ByteTxRx(self.portHandler, motor , self.ADDR_GOAL_CURRENT, self.MAX_CURRENT)

            while not maxxed:
                #will not hit the max threshold so we need to just keep moving until motion stops
                pos, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_ACTUAL_POSITION)
                if pos > 2147483647:
                    pos -= 4294967296

                if pos - prev_pos <= 0:
                    #limit has been hit
                    finger.closed_pos = pos
                    maxxed = True
                
                prev_pos = pos
                time.sleep(0.1)
                

            #move finger to open position
            finger.target_pos = finger.open_pos
            self.setPosition(motor, finger.open_pos)
            print(f"{name} is tensioned!")


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
        for i, finger in enumerate(self.hand.values()):
            position = finger.calculate_motor_angle(positions[i])



if __name__ == "__main__":
    motor = HandController()
    i = 0
    motor.close_port()
    print("Port closed")