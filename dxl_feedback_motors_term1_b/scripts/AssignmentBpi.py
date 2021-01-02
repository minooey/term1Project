#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This Script is written by Shahram M Sabery, RC2, Bartlett, UCL, DEC.2020
# The ROS side of  assignment B is done here, please note that there is 2 option for motor controls in unity
# and this option corresponds to scenario mode.
# please write the set_positionb in Unity Int32 publisher Topic to connect
# Thanks to Valentina, Georgia, Manios and Harvey for this fantastic course.

import os

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
import time
import rospy
from std_msgs.msg import Int32
from multiprocessing import Process, Lock
from dynamixel_sdk import *



# end default codes
class Controller:
    mutex = Lock()

    def __init__(self):
        # Control table address
        self.ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
        self.ADDR_PRO_GOAL_POSITION = 116  # Control table address for goal Position
        self.ADDR_PRO_PRESENT_POSITION = 132  # Control table address for present Position
        self.ADDR_PRO_PRESENT_TEMPRATURE = 146  # Control table address for present Temprature
        self.ADDR_PRO_PRESENT_LOAD = 126  # Control table address for present Load

        # Protocol version
        PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID = [1, 2, 3]  # Dynamixel ID for the three motors
        BAUDRATE = 57600  # Dynamixel default baudrate : 57600
        DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.x = 0
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        # Ros variables
        with self.mutex:
            self.set_1 = rospy.Subscriber("set_positionb", Int32, self.callback, queue_size=10, buff_size=2 ** 24)
            self.set_positionb = 0
            self.set_2 = rospy.Subscriber("set_pos2", Int32, self.callback2, queue_size=10, buff_size=2 ** 24)
            self.set_pos2 = 0
            self.set_3 = rospy.Subscriber("set_pos3", Int32, self.callback3, queue_size=10, buff_size=2 ** 24)
            self.set_pos3 = 0
            self.position_pub1 = rospy.Publisher("current_position", Int32, queue_size=10)
            self.position_pub2 = rospy.Publisher("current_position1", Int32, queue_size=10)
            self.position_pub3 = rospy.Publisher("current_position2", Int32, queue_size=10)
            self.temprature_pub1 = rospy.Publisher("readTemp1", Int32, queue_size=1)
            self.temprature_pub2 = rospy.Publisher("readTemp2", Int32, queue_size=1)
            self.temprature_pub3 = rospy.Publisher("readTemp3", Int32, queue_size=1)
            self.load_pub1 = rospy.Publisher("readLoad1", Int32, queue_size=1)
            self.load_pub2 = rospy.Publisher("readLoad2", Int32, queue_size=1)
            self.load_pub3 = rospy.Publisher("readLoad3", Int32, queue_size=1)
            time.sleep(.1)


        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def dxl_torque_enable(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[0],
                                                                       self.ADDR_PRO_TORQUE_ENABLE,
                                                                       self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque for ID %03d enabled" % self.DXL_ID[0])

        # Enable Dynamixel Torque M2
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[1],
                                                                       self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque for ID %03d enabled" % self.DXL_ID[1])

        # Enable Dynamixel Torque M3
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[2],
                                                                       self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque for ID %03d enabled" % self.DXL_ID[2])

    def dxl_write(self, motor_values1):

        # Write goal  position for ID 1
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[0],
                                                                       self.ADDR_PRO_GOAL_POSITION, motor_values1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def dxl_write2(self, motor_values2):

        # Write goal  position for ID 2
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[1],
                                                                       self.ADDR_PRO_GOAL_POSITION, motor_values2)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def dxl_write3(self, motor_values3):

        # Write goal  position for ID 3
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[2],
                                                                       self.ADDR_PRO_GOAL_POSITION, motor_values3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def dxl_torque_disable(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[0],
                                                                       self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("Torque for ID %03d disabled" % self.DXL_ID[0])

        # Disable Dynamixel Torque 2
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[1],
                                                                       self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("Torque for ID %03d disabled" % self.DXL_ID[1])

        # Disable Dynamixel Torque 3
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[2],
                                                                       self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("Torque for ID %03d disabled" % self.DXL_ID[2])

        # Close port
        self.portHandler.closePort()

        # Call back for 3 motors sceniaro defined based on Assignment B
        #self.x is the variable which comes from unity


    def callback(self, data):

        self.x = data.data
        self.set_positionb = abs(self.x-2000)       #M1  rotation, this value is same as grasshopper matrix & unity scripts
    def callback2(self, data):
        self.set_pos2 = self.set_positionb / 2      #M2 head rotation, this value is same as grasshopper matrix & unity scripts

    def callback3(self, data):
        self.set_pos3 = self.x + 2000               #M2 arm rotation , this value is same as grasshopper matrix & unity scripts


    def dxl_listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('dxl_listener', anonymous=True)

        print("Press ESC to quit!")

        # instead of spin(), keep node running until ESC is pressed
        while not rospy.core.is_shutdown():
            self.dxl_write(self.set_positionb)
            self.dxl_write2(self.set_pos2)
            self.dxl_write3(self.set_pos3)

            # Read present position for ID 1
            dxl_present_pos_1, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[0],
                                                                                             self.ADDR_PRO_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                # publish the position goal ID 1
            position_message = Int32()

            position_message.data = dxl_present_pos_1
            self.position_pub1.publish(position_message)

            # Read present position for ID 2
            dxl_present_pos_2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[1],
                                                                                             self.ADDR_PRO_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                # publish the position goal ID 2

            position_message.data = dxl_present_pos_2
            self.position_pub2.publish(position_message)

            # Read present position for ID 3
            dxl_present_pos_3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[2],
                                                                                             self.ADDR_PRO_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            # publish the position goal Id 3
            position_message.data = dxl_present_pos_3
            self.position_pub3.publish(position_message)

            # Read present Load for ID 1
            dxl_present_load_1, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                              self.DXL_ID[0],
                                                                                              self.ADDR_PRO_PRESENT_LOAD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                # publish the Load  ID 1
            load_message = Int32()

            load_message.data = dxl_present_load_1
            self.load_pub1.publish(load_message)

            # Read present load for ID 2
            dxl_present_pos_2, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[1],
                                                                                             self.ADDR_PRO_PRESENT_LOAD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                # publish the load ID 2

            load_message.data = dxl_present_pos_2
            self.load_pub2.publish(load_message)

            # Read present load for ID 3
            dxl_present_pos_3, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[2],
                                                                                             self.ADDR_PRO_PRESENT_LOAD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            # publish the load  ID 3
            load_message.data = dxl_present_pos_3
            self.load_pub3.publish(load_message)

            # Read present temprature for ID 1
            dxl_present_temprature_1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler,
                                                                                                    self.DXL_ID[0],
                                                                                                    self.ADDR_PRO_PRESENT_TEMPRATURE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                # publish the temprature  ID 1
            temprature_message = Int32()

            temprature_message.data = dxl_present_temprature_1
            self.temprature_pub1.publish(temprature_message)

            # Read present temprature for ID 2
            dxl_present_pos_2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[1],
                                                                                             self.ADDR_PRO_PRESENT_TEMPRATURE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                # publish the temprature ID 2

            temprature_message.data = dxl_present_pos_2
            self.temprature_pub2.publish(temprature_message)

            # Read present temprature for ID 3
            dxl_present_pos_3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler,
                                                                                             self.DXL_ID[2],
                                                                                             self.ADDR_PRO_PRESENT_TEMPRATURE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            # publish the temprature  ID 3
            temprature_message.data = dxl_present_pos_3
            self.temprature_pub3.publish(temprature_message)



if __name__ == '__main__':
    c = Controller()
    # enable torque for both motors
    c.dxl_torque_enable()
    c.dxl_listener()
    c.dxl_torque_disable()
