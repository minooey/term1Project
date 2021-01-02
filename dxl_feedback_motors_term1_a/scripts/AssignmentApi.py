#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Script is written by Shahram M Sabery, RC2, Bartlett, UCL, DEC.2020
# The ROS side of assignment A is done here
# please write the set_pwm in Unity Int32 publisher Topic to connect
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

import rospy
from std_msgs.msg import Int32, Bool
from dynamixel_sdk import *

# end default codes

# Control table address
ADDR_OPERATING_MODE    = 11		  # The Control table address for operating
ADDR_PRO_TORQUE_ENABLE = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_PWM      = 100              # Control table address for goal PWM
ADDR_PRO_PRESENT_PWM   = 124              # Control table address for present PWM

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = [1, 2, 3]             # Dynamixel ID for the three motors
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

motor1_pwm = 0
motor2_pwm = 0
motor3_pwm = 0


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def dxl_operating_mode(operating_mode_value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_OPERATING_MODE, operating_mode_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM operating mode for ID %03d enabled" % DXL_ID[0])

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_OPERATING_MODE, operating_mode_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM operating mode for ID %03d enabled" % DXL_ID[1])

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[2], ADDR_OPERATING_MODE, operating_mode_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM operating mode for ID %03d enabled" % DXL_ID[2])


def dxl_torque_enable():
   # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque for ID %03d enabled" % DXL_ID[0])

    # Enable Dynamixel Torque M2
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque for ID %03d enabled" % DXL_ID[1])

  # Enable Dynamixel Torque M3
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[2], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque for ID %03d enabled" % DXL_ID[2])

def dxl_write(motor_values):
    motor_pwm_value_1 = motor_values[0]
    motor_pwm_value_2 = motor_values[1]
    motor_pwm_value_3 = motor_values[2]

    # Write goal pwm position for ID 1
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_GOAL_PWM, motor_pwm_value_1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write goal pwm position for ID 2
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_GOAL_PWM, motor_pwm_value_2)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

 # Write goal pwm position for ID 3
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[2], ADDR_PRO_GOAL_PWM, motor_pwm_value_3)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))



def dxl_torque_disable():
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("Torque for ID %03d disabled" % DXL_ID[0])

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("Torque for ID %03d disabled" % DXL_ID[1])

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[2], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("Torque for ID %03d disabled" % DXL_ID[2])

    # Close port
    portHandler.closePort()

    #Call back for 3 motors sceniaro defined based on Assignment A
def callback(data):
    global motor2_pwm, motor3_pwm

    dxl_write([data.data, motor2_pwm, motor3_pwm])
    motor1_pwm = dxl_read()

    if motor1_pwm > 200:
        motor2_pwm = motor1_pwm - 200
        motor3_pwm = 0
        dxl_write([motor1_pwm, motor2_pwm, motor3_pwm])

    if motor1_pwm > 300:
        motor2_pwm = motor1_pwm - 200
        motor3_pwm = motor1_pwm - 300
        dxl_write([motor1_pwm, motor2_pwm, motor3_pwm])

    else:
        motor2_pwm = 0
        motor3_pwm = 0
        dxl_write([motor1_pwm, motor2_pwm, motor3_pwm])



def dxl_read():
    # Read present position for ID 1
    dxl_present_pwm_1, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read present position for ID 2
    dxl_present_pwm_2, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read present position for ID 3
    dxl_present_pwm_3, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID[2], ADDR_PRO_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


    print("Motors PWM are %d , %d and %d. Pres 'Ctrl+c' to exit." % (dxl_present_pwm_1, dxl_present_pwm_2, dxl_present_pwm_3))

    return dxl_present_pwm_1


def dxl_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('dxl_listener', anonymous=True)

    rospy.Subscriber("set_pwm", Int32, callback, queue_size=1, buff_size=2**24)

    print("Press ESC to quit!")
    #instead of spin(), keep node running until ESC is pressed
    while not rospy.core.is_shutdown():

        rate = rospy.Rate(1000)  # 10hz
        rate.sleep()

if __name__ == '__main__':

    dxl_operating_mode(16) # this is the operating mode value for PWM

    #enable torque for both motors
    dxl_torque_enable()

    dxl_listener()

    dxl_torque_disable()
