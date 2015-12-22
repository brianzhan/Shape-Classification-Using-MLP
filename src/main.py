#!/usr/bin/env python
#import roslib
#import rospy
import math
#from fw_wrapper.srv import *
import time
import sys
import pickle
import signal
from copy import copy
from neuralnet import *
from neuralnet import NeuralNetwork
from scipy import interpolate
import os, os.path
import numpy as np

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# ******************************************************************************************
# --------------------------------Built-In Commands-----------------------------------------
# wrapper function to call service to get sensor value

def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_multiple_wheel_torque(motor_ids, target_values):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', 0, 0, len(motor_ids), motor_ids, target_values)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_multi_wheel_speed(motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync',0,0,len(motor_ids),motor_ids,target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# @Type: Void, for debugging
def ReadAllMotors():
    m1 = getMotorPositionCommand(10)
    m2 = getMotorPositionCommand(11)
    m3 = getMotorPositionCommand(12)
    m4 = getMotorPositionCommand(13)
    m5 = getMotorPositionCommand(14)
    rospy.loginfo("Motor positions: %d, %d, %d, %d, %d", m10, m11, m12, m13, m14)

# --------------------------------End Built-In Commands-------------------------------------
# ******************************************************************************************

# -------------------------------- Start Motor Commands ------------------------------------
def ReadDMSvalue():
    DMSvalue = getSensorValue(DMSport)
    return DMSvalue

def setDMSstraight():
    setMotorMode(10, 0)
    setMotorTargetPositionCommand(10, 512)
    setMotorTargetPositionCommand(3, 512)

def setplatformstraight():
    setMotorMode(9, 0)
    setMotorTargetPositionCommand(9, 515)

def rotatePlatform():
    setMotorMode(9, 1)
    motor_ids = 9
    target_values = 1100
    setMotorTargetSpeed(motor_ids, target_values)

def checkfile():
    current = os.getcwd()
    wpath = current + "/data/" + time.strftime("%H%M%S")
    return wpath

def rotatesave():
    rotatePlatform()
    wpath = checkfile()
    fh = open(wpath, "a")
    starttime = time.time()
    timerunning = time.time() - starttime
    i=0
    while i<520:
        DMSvalue = ReadDMSvalue()
        timerunning = time.time() - starttime
        DMSvalue = str(DMSvalue)
        timerec = str(timerunning)
        print(DMSvalue)
        fh.write(DMSvalue)
        fh.write("    ")
        fh.write(timerec)
        fh.write("\n")
        i+=1
    fh.close
    stopr()
    return wpath

def StopAllMotors():
    motor_ids = (3, 10, 9)
    target_values = (1024, 1024, 1024, 1024)
    set_multiple_wheel_torque(motor_ids, target_values)

def stopr():
    motor_ids = 9
    target_values = 1024
    setMotorTargetSpeed(motor_ids, target_values)

# Global variables:
DMSport = 4

global theta
global unvisited

# Main function
if __name__ == "__main__":
   # rospy.init_node('example_node', anonymous=True)
   # rospy.loginfo("Starting Group L Control Node...")
   # r = rospy.Rate(10) # 10hz
   # stopr()

    mode = raw_input("Enter mode (Train 1) or (Collect and Identify 2)) ")
    mode = int(mode)

    if mode is 1:
        train_net()
    elif mode is 2:
        setDMSstraight()
        setplatformstraight()   
        rospy.sleep(3)

        wpath = rotatesave()
        for testset in range(1,6):
            demo(wpath,testset)
    else:
        print("Wrong Input")
