#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *
import time

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

# wrapper function to call service to get sensor value
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
def sitDown():
    target_val = 700
    target_val2 = 324
    setMotorTargetPositionCommand(1, 512) #head centers
    setMotorTargetPositionCommand(6, 124) #leg sits
    setMotorTargetPositionCommand(7, 900) #leg sits
    setMotorTargetPositionCommand(2, 512) #arm centers
    setMotorTargetPositionCommand(4, 512) #arm centers
    time.sleep(.5)

def nuzzleHand():
    setMotorTargetPositionCommand(2, 512) #arm centers
    setMotorTargetPositionCommand(4, 512) #arm centers
    setMotorTargetPositionCommand(1, 580) #head moves left
    time.sleep(.5)
    setMotorTargetPositionCommand(1, 444) #head moves right
    time.sleep(.5)

def standup():
    for num in range(6,8):
        setMotorTargetPositionCommand(num, 512) #stands back legs

def wave():
    setMotorTargetPositionCommand(1, 512) #centers head
    setMotorTargetPositionCommand(2, 720) #sets limb
    setMotorTargetPositionCommand(4, 200) #sets hand
    time.sleep(.5)
    setMotorTargetPositionCommand(4, 400) #waves hand
    time.sleep(.5)
    

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group D Control Node...")
    
    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
        
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 1
        sensor_reading = getSensorValue(port)
        rospy.loginfo("Sensor value at port %d: %f", 1, sensor_reading)

        # call function to set motor position
        if sensor_reading == 0: #robot will sit
            sitDown()
        elif sensor_reading >= 600:
            nuzzleHand()
        else:
            standup()
            wave()
           
        # sleep to enforce loop rate
        r.sleep()
    

