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
    

def rightLegForward():
	#Right Leg Forward, Left Leg back
	setMotorTargetPositionCommand(2, 590)#match 3 left forward
	setMotorTargetPositionCommand(4, 275)#match 5 left forward
	setMotorTargetPositionCommand(6, 475)#match 7 left forward
	setMotorTargetPositionCommand(3, 600)
	setMotorTargetPositionCommand(5, 900)
	setMotorTargetPositionCommand(7, 480)

def leftLegForward():
	#Left Leg forward, Right leg back
	setMotorTargetPositionCommand(3, 433)#match 2 right forward
	setMotorTargetPositionCommand(5, 749)#match 4 right forward
	setMotorTargetPositionCommand(7, 549)#match 6 right forward
	setMotorTargetPositionCommand(2, 424)
	setMotorTargetPositionCommand(4, 124)
	setMotorTargetPositionCommand(6, 544)

def rightRotate1():
	#Right Leg Forward, Left Leg back
	setMotorTargetPositionCommand(2, 590)
	setMotorTargetPositionCommand(4, 275)
	setMotorTargetPositionCommand(6, 475)

def rightRotate2():
	#Left Leg forward, Right leg back
	setMotorTargetPositionCommand(2, 424)
	setMotorTargetPositionCommand(4, 124)
	setMotorTargetPositionCommand(6, 544)

def leftRotate1():
	#Right Leg Forward, Left Leg back
	setMotorTargetPositionCommand(3, 600)
	setMotorTargetPositionCommand(5, 900)
	setMotorTargetPositionCommand(7, 480)


def leftRotate2():
	#Left Leg forward, Right leg back
	setMotorTargetPositionCommand(3, 433)
	setMotorTargetPositionCommand(5, 749)
	setMotorTargetPositionCommand(7, 549)

def turnLeft():
	rightRotate1()
#	time.sleep(0.2)
	setMotorTargetPositionCommand(4, 490)
	rightRotate2()

def turnRight():
	leftRotate2()
#	time.sleep(0.2)
	setMotorTargetPositionCommand(5, 490)
	leftRotate1()		
# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group D Control Node...")
    
    # control loop running at 30hz
    r = rospy.Rate(30) # 30hz
    i = 0       
    while not rospy.is_shutdown():
        # call function to get sensor value
        portLeft = 1
	portFront = 6
	portRight = 5

	sensor_readingLeft = getSensorValue(portLeft)
	sensor_readingFront = getSensorValue(portFront)
	sensor_readingRight = getSensorValue(portRight)
	rospy.loginfo("Sensor value at port %d: %f", 1, sensor_readingLeft)
	rospy.loginfo("Sensor value at port %d: %f", 5, sensor_readingRight)
	rospy.loginfo("Sensor value at port %d: %f", 6, sensor_readingFront)
#Setting default position, to be used when only the rotation commands are being done
	#setMotorTargetPositionCommand(1, 512)
	#setMotorTargetPositionCommand(2, 512)
	#setMotorTargetPositionCommand(4, 350)
	#setMotorTargetPositionCommand(6, 512)
	#etMotorTargetPositionCommand(3, 512)
	#setMotorTargetPositionCommand(5, 775)
	#setMotorTargetPositionCommand(7, 512)

#Walking forward
	if  sensor_readingFront <= 1000:
		rightLegForward()
		setMotorTargetPositionCommand(4, 484)
		leftLegForward()
		setMotorTargetPositionCommand(5, 540)

#If any of the side sensors detect a wall on either side, it will turn away from it		
	elif sensor_readingRight > 75:
		turnLeft()
		time.sleep(.1)
		turnLeft()
		time.sleep(.1)
		turnLeft()
		time.sleep(.1)
		turnLeft()
	elif sensor_readingLeft > 75:
		turnRight()
		time.sleep(.1)
		turnRight()
		time.sleep(.1)
		turnRight()
		time.sleep(.1)
		turnRight()
		
	#Turning 90 degrees to the right
	#while i < 13:
	#	turnRight()
	#	i+=1
	#Turning 90 degrees to the left
	#while i < 13:
	#	turnLeft()
	#	i+=1

	#Turning 180 degrees
	#while i < 26:
	#	turnLeft()
	#	i+=1


if start == end:
		
		return 
	else:
		visited.append(start)
		for direction in range(1, 5):
			blocked = getNeighborObstacle(start[0], start[1], direction)
			if blocked == 0:
				if direction == 1:
					x = start[0] -1, start[1]
					neighbors.append(x)
				elif direction == 2:
					x = start[0], start[1] + 1
					neighbors.append(x)
				elif direction == 3:
					x = start[0] + 1, start[1]
					neighbors.append(x)
				elif direction == 4:
					x = start[0], start[1] - 1
					neighbors.append(x)
		for neighbor in neighbors:
			if neighbor not in visited:
				for i in range(1,5):
					if currentDirection == i:
						currentCost = getCost(start[0], start[1])
						neighborCost = currentCost + 1
						setNeighborCost(start[0], start[1], i, neighborCost)

					else:
						currentCost = getCost(start[0], start[1])	
						neighborCost = currentCost + 1.5
						setNeighborCost(start[0], start[1], i, neighborCost)
					findPath(neighbor, end, i)
	return
	
	

	

#Assignment1
        # call function to set motor position
        #if sensor_reading == 0: #robot will sit
        #    sitDown()
        #elif sensor_reading >= 600:
        #    nuzzleHand()
        #else:
        #    standup()
        #    wave()
           
        # sleep to enforce loop rate
        r.sleep()
    

