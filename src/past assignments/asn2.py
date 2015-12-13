#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *
from map import *
from random import randint 

#Lauren DeNaut
#Pascal Egli

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

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
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

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
            
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
        
# initialize the robot position       
def initializeRobot():
    initialArray = [(1,512), (2,512), (3,512), (4,512), (5,800), (6,224), (7,524), (8,500), (9,512)]
    i = 0
    while not rospy.is_shutdown() and i < 9:
        setMotorTargetPositionCommand(initialArray[i][0], initialArray[i][1]) #initialize
        i += 1

        r.sleep()

# make the robot walk      
def walk():
	print("inside walk")
	#commandArray = [(6,500), (8,290),(4,610),(3,800), (1,560), (2,560),(4, 512), (3,512), (6, 400), (5,524), (7,734), (3,414), (4,224),(2,464), (1,464), (3,512),(4,512), (5, 624)] 
	#commandArray = [(4,580),(3,710), (1,610), (2,610),(4, 512), (3,512), (3,444), (4,314),(2,414), (1,414), (3,512),(4,512)] 
	commandArray = [(3,710),(4,580), (1,610), (2,610), (9,419), (4, 512), (3,512), (4,314),(3,444),(2,419), (1,419), (9,610), (3,512),(4,512)]
	i=0
	while not rospy.is_shutdown() and i < len(commandArray):
		#if i == 0 or i == 9: #three at a time
		#if i == 0 or i==2 or i == 4 or i==6 or i==8 or i==10: #two at a time
		
		if i==0 or i==6:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			rospy.sleep(0.1)
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			
		if i==5 or i ==12:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#if i==6 or i==13:
			rospy.sleep(1)
			
		if i==2 or i==9:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#if i==4 or i==11:
			rospy.sleep(1)
			
			#i+=1
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#i+=1
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		#elif i==6 or i==15:
			#increment(commandArray[i][0], commandArray[i][1])
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#i+=1
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		else:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		i+=1


		r.sleep()
		
def walk2():
	#commandArray = [(6,500), (8,290),(4,610),(3,800), (1,560), (2,560),(4, 512), (3,512), (6, 400), (5,524), (7,734), (3,414), (4,224),(2,464), (1,464), (3,512),(4,512), (5, 624)] 
	#commandArray = [(4,580),(3,710), (1,610), (2,610),(4, 512), (3,512), (3,444), (4,274),(2,414), (1,414), (3,512),(4,512)] 
	commandArray = [(4,580),(3,710), (1,610), (2,610),(4, 512), (3,512), (3,444), (4,314),(2,409), (1,409), (3,512),(4,512)]
	i=0
	while not rospy.is_shutdown() and i < len(commandArray):
		#if i == 0 or i == 9: #three at a time
		#if i == 0 or i==2 or i == 4 or i==6 or i==8 or i==10: #two at a time
		if i%2==0:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#rospy.loginfo("i%d", i)
			if i==3 or i==9:
				rospy.loginfo("Sleep")
				rospy.sleep(1)
			#i+=1
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#i+=1
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		#elif i==6 or i==15:
			#increment(commandArray[i][0], commandArray[i][1])
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#i+=1
			#setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		else:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		i+=1


		r.sleep()


#walk along wall on left side
def walkStraightLeft():
    commandArray = [(6,500), (8,290),(4,610),(3,800), (3,512), (1,580), (2,580),(4, 512), (6, 400), (5,524), (7,734), (3,414), (4,224), (4,512),(2,444), (1,444), (3,512), (5, 624)]
    i=0
    while not rospy.is_shutdown() and i < len(commandArray):
	    if i == 0:
		    side, change = controlerLeft()
		    if side == "right":
			    commandArray[5] = (1, change)
			    commandArray[6] = (2, change)
			    commandArray[14] = (2, 444)
			    commandArray[15] = (1, 444)
		    else:
			    commandArray[14] = (2, change)
			    commandArray[15] = (1, change)
			    commandArray[5] = (1, 580)
			    commandArray[6] = (2, 580)
		
		
	    if i == 0 or i == 9: #three at a time
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		    i+=1
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		    i+=1
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
	
	    elif i==6 or i==15:
		    increment(commandArray[i][0], commandArray[i][1])
	    else:
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
	    i+=1

	    r.sleep()


#walk along wall on right side
def walkStraightRight():
    commandArray = [(6,500), (8,290),(4,610),(3,800), (3,512), (1,580), (2,580),(4, 512), (6, 400), (5,524), (7,734), (3,414), (4,224), (4,512),(2,444), (1,444), (3,512), (5, 624)]
    i=0
    while not rospy.is_shutdown() and i < len(commandArray):
	    if i == 0:
		    side, change = controlerRight()
		    if side == "right":
			    commandArray[5] = (1, change)
			    commandArray[6] = (2, change)
			    commandArray[14] = (2, 444)
			    commandArray[15] = (1, 444)
		    else:
			    commandArray[14] = (2, change)
			    commandArray[15] = (1, change)
			    commandArray[5] = (1, 580)
			    commandArray[6] = (2, 580)
		
		
	    if i == 0 or i == 9: #three at a time
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		    i+=1
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		    i+=1
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
	
	    elif i==6 or i==15:
		    increment(commandArray[i][0], commandArray[i][1])
	    else:
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
	    i+=1

	    r.sleep()

#walk and react to being blocked
def walkBlocked():
    commandArray = [(6,500), (8,290),(4,610),(3,800), (3,512), (1,580), (2,580),(4, 512), (6, 400), (5,524), (7,734), (3,414), (4,224), (4,512),(2,444), (1,444), (3,512), (5, 624)]
    i=0
    while not rospy.is_shutdown() and i < len(commandArray):
	    if i == 0:
		    sensLeft = getSensorValue(3)
		    rospy.loginfo("Left sensor value %d", sensLeft)
		    sensRight = getSensorValue(5)
		    rospy.loginfo("Right sensor value %d", sensRight)
		    sensFront = getSensorValue(4)
		    rospy.loginfo("Front sensor value %d", sensFront)

		    if sensFront>1000 and sensLeft>=20 and sensRight>=20: #turn around
			    rospy.loginfo("Turn around")
			    turn180Right()
			    rospy.sleep(10)
		    elif sensFront>1000 and sensRight>20: #turn left
			    rospy.loginfo("Turn left")
			    turnLeft()
			    rospy.sleep(5)
		    elif sensFront>1000 and sensLeft>20: #turn right
			    rospy.loginfo("Turn right")
			    turnRight()
			    rospy.sleep(5)
		
		
	    if i == 0 or i == 9: #three at a time
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		    i+=1
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
		    i+=1
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
	
	    elif i==6 or i==15:
		    increment(commandArray[i][0], commandArray[i][1])
	    else:
		    setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
	    i+=1

	    r.sleep()

#calculate values to correct on left side
def controlerLeft():
    sensLeft = getSensorValue(3)
    rospy.loginfo("Left sensor value %d", sensLeft)

    desiredDistance = 40 #sensor value at 15cm away
    Kp = 0.75

    if sensLeft < 40:
	    change = min(524, (444 + (desiredDistance-sensLeft)*Kp))
	    rospy.loginfo("change is left %d", change)
	    return ("left", change)
    else: 
	    change = max(500, (580 - (sensLeft-desiredDistance)*Kp))
	    rospy.loginfo("change is right %d", change)
	    return ("right", change)

#calculate values to correct on right side
def controlerRight():
    sensRight = getSensorValue(5)
    rospy.loginfo("Right sensor value %d", sensRight)

    desiredDistance = 40 #sensor value at 15cm away
    Kp = 0.75

    if sensRight >= 40: 
	    change = min(524, (444 + (sensRight-desiredDistance)*Kp))
	    rospy.loginfo("change is left %d", change)
	    return ("left", change)
    else: 
	    change = max(500, (580 - (desiredDistance-sensRight)*Kp)) 
	    rospy.loginfo("change is right %d", change)
	    return ("right", change)


def increment(motor, position):
    current = getMotorPositionCommand(motor)
    if current < position:
	    while not rospy.is_shutdown() and current < position:
		    setMotorTargetPositionCommand(motor, current+15)
		    current += 15
		    r.sleep()
    else:
	    while not rospy.is_shutdown() and current > position:
		    setMotorTargetPositionCommand(motor, current-15)
		    current -= 15
		    r.sleep()


# make the robot turn right
def turnRight():
    #commandArray = [(3,710),(4,580),(1,585), (2,439), (4,512),(3,512), (4,314),(3,444), (2,512), (1,512), (3,512), (4,512)]
    print("turn right")
    commandArray = [(3,710),(4,580),(1,580), (2,444), (4,512),(3,512), (4,314),(3,444), (2,512), (1,512), (3,512), (4,512)]
    i=0
    count=1
    
    while not rospy.is_shutdown() and i < len(commandArray):
		#if i == 0 or i == 9: #three at a time
		#if i == 0 or i==2 or i == 4 or i==6 or i==8 or i==10: #two at a time
		
		if i%2 == 0:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#if i==5 or i==11:
				#rospy.sleep(0.5)
		else:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			
		rospy.sleep(0.5)	
		i+=1
		
		if i >= len(commandArray) and count<3:
			count += 1
			i=0
		    
		r.sleep()
		
# make the robot turn right
def correctRight():
    #commandArray = [(3,710),(4,580),(1,585), (2,439), (4,512),(3,512), (4,314),(3,444), (2,512), (1,512), (3,512), (4,512)]
    commandArray = [(3,710),(4,580),(1,540), (2,484), (4,512),(3,512), (4,314),(3,444), (2,512), (1,512), (3,512), (4,512)]
    i=0
    count=1
    
    while not rospy.is_shutdown() and i < len(commandArray):
		#if i == 0 or i == 9: #three at a time
		#if i == 0 or i==2 or i == 4 or i==6 or i==8 or i==10: #two at a time
		
		if i%2 == 0:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#if i==5 or i==11:
				#rospy.sleep(0.5)
		else:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			
		rospy.sleep(0.5)	
		i+=1
		
		if i >= len(commandArray) and count<1:
			count += 1
			i=0
		    
		r.sleep()

# make the robot turn left 
def turnLeft(): 
    #commandArray = [(5,524), (7,734), (3,414), (4,224), (4,512), (2,447), (1,577), (3,512), (6,559), (8,244), (4,630), (3,800), (3,512), (1,512), (2,512), (4,512)]
	#commandArray = [(3,414), (4,224), (4,512), (2,447), (1,577), (3,512),(4,630), (3,800), (3,512), (1,512), (2,512), (4,512)]
	print("currLeft")
	commandArray = [(4,314),(3,444), (2,453),(1,572), (3,512), (4,512), (3,710), (4,580),  (1,512), (2,512), (4,512),(3,512)]
	i=0
	count=1
	while not rospy.is_shutdown() and i < len(commandArray):
		if i%2 == 0:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#if i==5 or i==11:
				#rospy.sleep(0.5)
		else:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			
		rospy.sleep(0.5)	
		i+=1
		
		if i >= len(commandArray) and count<3:
			count += 1
			i=0
		    
		r.sleep()
		
# make the robot turn left 
def correctLeft(): 
    #commandArray = [(5,524), (7,734), (3,414), (4,224), (4,512), (2,447), (1,577), (3,512), (6,559), (8,244), (4,630), (3,800), (3,512), (1,512), (2,512), (4,512)]
	#commandArray = [(3,414), (4,224), (4,512), (2,447), (1,577), (3,512),(4,630), (3,800), (3,512), (1,512), (2,512), (4,512)]
	commandArray = [(4,314),(3,444), (2,483),(1,542), (3,512), (4,512), (3,710), (4,580),  (1,512), (2,512), (4,512),(3,512)]
	i=0
	count=1
	while not rospy.is_shutdown() and i < len(commandArray):
		if i%2 == 0:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			i+=1
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			#if i==5 or i==11:
				#rospy.sleep(0.5)
		else:
			setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
			
		rospy.sleep(0.5)	
		i+=1
		
		if i >= len(commandArray) and count<1:
			count += 1
			i=0
		    
		r.sleep()


# make the robot turn 180 degrees (left direction)
def turn180Left():
    for i in range(0,2):
	    turnLeft()

# make the robot turn 180 degrees (left direction)
def turn180Right():
    for i in range(0,2):
	    turnRight()


# make the robot pose like a ballerina
def ballerina():
    commandArray = [(3,424), (4,224), (5,1000), (6,24), (4,350)]
    i = 0
    while not rospy.is_shutdown() and i < len(commandArray):
        setMotorTargetPositionCommand(commandArray[i][0], commandArray[i][1])
        if i != len(commandArray)-1:
            setMotorTargetPositionCommand(commandArray[i+1][0], commandArray[i+1][1])
        i += 2
        
        r.sleep()


#### Input: Current orientation 0=north, west =1, south =2, east =3
#### Returns: Change of i and j

def positionChange(Orientation): #keeps track of where I am 
	if Orientation == 0: #facing north
		return (0 , -1)
		
	elif Orientation ==1: #facing west
		return (1, 0)
	
	elif Orientation ==2: #facing south
		return (0, 1)
		
	elif Orientation ==3: #facing east
		return (-1 , 0)

def checkSides(i, j, myMap, val, start):
	print("inside checksides for", i, j)
	if (i-1!=start[0] or j!=start[1]) and myMap.getCost(i-1, j) == 0 and myMap.getNeighborObstacle(i, j, DIRECTION.North) == 0:
		myMap.setCost(i-1, j, val)

	if (i!=start[0] or j-1!=start[1]) and myMap.getCost(i, j-1) == 0 and myMap.getNeighborObstacle(i, j, DIRECTION.West) == 0:
		myMap.setCost(i, j-1, val)

	if (i!=start[0] or j+1!=start[1]) and myMap.getCost(i, j+1) == 0 and myMap.getNeighborObstacle(i, j, DIRECTION.East) == 0:
		myMap.setCost(i, j+1, val)

	if (i+1!=start[0] or j!=start[1]) and myMap.getCost(i+1, j) == 0 and myMap.getNeighborObstacle(i, j, DIRECTION.South) == 0:
		myMap.setCost(i+1, j, val)
	
	return
		
def calcMapCost(start, myMap, end):
	myMap.printObstacleMap()
	print("before", myMap.costMap)
	checkSides(start[0], start[1], myMap, 1, start)
	print("after", myMap.costMap)

	currCost = 1
	cont = 1
	print(len(myMap.costMap))
	print(len(myMap.costMap[0]))
	while cont == 1:
		for i in range(0, len(myMap.costMap)): #rows
			for j in range(0, len(myMap.costMap[0])): #cols
				if myMap.getCost(i,j) == currCost:
					checkSides(i, j, myMap, currCost+1, start)
		
		if myMap.getCost(end[0], end[1]) != 0: 
			cont = 0
		#for a in range(0, len(myMap.costMap)):
			#for b in range(0, len(myMap.costMap[0])):
				#if myMap.costMap[a][b] == 0 and (a!= start[0] or b!= start[1]):
				#if myMap.costMap[a][b] == 0:
					#if a!=start[0] or b!=start[1]:
						#cont = 1
						#print("go again")
						#break
		
					
		currCost +=1
	return 

def getMinCell(myMap, curr, start, currCost):
	print("getmincell")
	goTo = curr
	minCost = float("inf")
	i = curr[0]
	j = curr[1]
	print("curr cost", currCost)
	if (myMap.getCost(i-1, j) == currCost-1) and myMap.getNeighborObstacle(i, j, DIRECTION.North) == 0:
		if myMap.getCost(i-1, j) < minCost:
			minCost = myMap.getCost(i-1, j)
			goTo = (i-1, j)
			print("minCos", minCost)
			print("goTo", goTo)

	if  (myMap.getCost(i, j-1) == currCost-1) and myMap.getNeighborObstacle(i, j, DIRECTION.West) == 0:
		if myMap.getCost(i, j-1) < minCost:
			minCost = myMap.getCost(i, j-1)
			goTo = (i, j-1)
			print("minCos", minCost)
			print("goTo", goTo)

	if (myMap.getCost(i, j+1) ==currCost-1) and myMap.getNeighborObstacle(i, j, DIRECTION.East) == 0:
		if myMap.getCost(i, j+1) < minCost:
			minCost = myMap.getCost(i, j+1)
			goTo = (i, j+1)
			print("minCos", minCost)
			print("goTo", goTo)

	if (myMap.getCost(i+1, j) ==currCost-1) and myMap.getNeighborObstacle(i, j, DIRECTION.South) == 0:
		if myMap.getCost(i+1, j) < minCost:
			minCost = myMap.getCost(i+1, j)
			goTo = (i+1, j)
			print("minCos", minCost)
			print("goTo", goTo)
	return goTo

def calcTurns(goTo, curr, currOri):
	countWalk = 6
	print("inside calcturns")
	d = {}
	d[0] = "North"
	d[1] = "West"
	d[2] = "South"
	d[3] = "East"
	d[-1] = "no"
	
	direction = -1
	print("curr", curr)
	print("go to", goTo)
	print("currOri", currOri)
	if goTo[0] > curr[0]: #i+1, South
		direction = 2 
	elif goTo[0] < curr[0]: #i-1, North
		direction = 0
	elif goTo[1] > curr[1]: #j+1, East
		direction = 3
	elif goTo[1] < curr[1]: #j-1, West
		direction = 1
	
	print("calc turns direction", direction, d[direction])
	if direction != -1:
		if currOri > direction: #turn right
			print("currOri is", currOri)
			print("direction is", direction)
			turns = currOri-direction
			print("turns", turns)
			if turns == 3: #switch, turn left once
				print("turn left")
				turnLeft()
				countWalk -=1
			else:
				countWalk -=1
				for i in range(0, turns):
					turnRight()	
					print("turn right") 
		else: #turn left
			turns = direction-currOri
			if turns == 3: #switch, turn right once
				print("turn right")
				countWalk -=1
				turnRight()
			else:
				countWalk-=1
				for i in range(0, turns):
					print("turn left")
					turnLeft()
		return (direction, countWalk)
	
	return (currOri, countWalk)

def getPath(myMap, start, end):
	print("getpath")
	q = []
	#initialStart = start
	currCost = myMap.getCost(start[0], start[1])
	count=0
	while start != end:
	#while count != 4:
		print("start", start)
		print("end", end)
		goTo = getMinCell(myMap, start, end, currCost)
		q.append(goTo)
		start = goTo
		print("start", start)
		currCost-=1
		count+=1
		#start=end #DELETE
	print("q", q)
	return(q)
	

def navigateMap(myMap, start, end, currOri):
	curr = start

	q = getPath(myMap, start, end)
	#q = [(0,1), (0,2), (1,2), (2,2)]
	#q = [(1,0), (2,0), (2,1), (2,2)]
	print("q", q)
	#goTo = getMinCell(myMap, curr)
	#direction = calcTurns(goTo, curr, currOri) #turn
	#for i in range(0, 6):
		#walk()
		#if goTo != end:
			#navigateMap(myMap, goTo, end, direction)
		#return
	while len(q) > 0:
		goTo = q.pop(0)
		print("go to is", goTo)
		currOri, walkCount = calcTurns(goTo, curr, currOri) #turn
		curr = goTo
		for i in range(0, walkCount):
			walk()
		r.sleep()
		
	return currOri
		
def getTheirDir(ourDir):
	if ourDir == 0 or ourDir == 2:
		theirDir = ourDir+1 #3 or 1
	elif ourDir == 3:
		theirDir = ourDir -1 #2
	else: #1
		theirDir = ourDir + 3 #4
		
	return theirDir
		
		
def wander(myMap, visitedMap, curr, currOri, count):
	d = {}
	d[0] = "North"
	d[1] = "West"
	d[2] = "South"
	d[3] = "East"
	
	print("inside wander")
	#check 3 sides
	sensLeft = getSensorValue(3)
	print("sensLeft", sensLeft)
	sensRight = getSensorValue(5)
	print("sensRight", sensRight)
	sensFront = getSensorValue(4)
	print("sensFront", sensFront)
	thresLeft = 0
	thresRight = 0
	thresFront = 1300
	walkThresL = 610
	walkThresR = 705
	walkThresF = 1800
	moves = []
	
	
	if (sensLeft > thresLeft) or (curr[0]==0 and currOri==3) or (curr[1]==0 and currOri==0) or (curr[1]==7 and currOri==2) or (curr[0]==7 and currOri==1):
		blockedL = 1
		print("left blocked")
	else:
		moves.append("left")
		blockedL = 0
	ourDir = (currOri+1)%4 
	theirDir = getTheirDir(ourDir)
	myMap.setObstacle(curr[0],curr[1], blockedL, theirDir)	#new


	if (sensRight > thresRight) or (curr[0]==0 and currOri==1) or (curr[1]==0 and currOri==2) or (curr[1]==7 and currOri==0) or (curr[0]==7 and currOri==3):
		blockedR = 1
		print("right blocked")
	else:
		moves.append("right")
		blockedR = 0	
	ourDir = (currOri-1)%4 
	theirDir = getTheirDir(ourDir)
	myMap.setObstacle(curr[0],curr[1], blockedR, theirDir) #new
		
		
	if (sensFront >= thresFront) or (curr[0]==0 and currOri==0) or (curr[1]==0 and currOri==1) or (curr[1]==7 and currOri==3) or (curr[0]==7 and currOri==2):
		blockedF = 1
		print("front blocked")
	else:
		moves.append("front")
		blockedF = 0
	ourDir = currOri
	theirDir = getTheirDir(ourDir)
	myMap.setObstacle(curr[0],curr[1], blockedF, theirDir)	#new
	
	
	myMap.printObstacleMap() #Prints current obstacle Map 	#new
	
	#check if already visited and decrease chance of revisit
	
	if len(moves) > 0:
		moveIndex = randint(0,len(moves)-1)
	else:
		moveIndex = -1
	print("moves", moves)
	print("random move index", moveIndex)
	if moveIndex != -1:
		print("will move", moves[moveIndex])
	#moveDir = moves[moveIndex]
	
	#get cell i,j
	if moveIndex == -1: #turn around, blocked
		moveDir = (currOri+2)%4
	elif moves[moveIndex] == "left":
		moveDir = (currOri+1)%4
	elif moves[moveIndex] == "right":
		moveDir = (currOri-1)%4
	else: #moveDir=="front"
		moveDir = currOri
		
	print("currOri", currOri, d[currOri])
	print("moveDir", moveDir, d[moveDir])
	
		
	if moveDir == 0:
		goTo = (curr[0]-1, curr[1]) #North i-1,j
	elif moveDir == 1: 
		goTo = (curr[0], curr[1]-1) #West, i, j-1
	elif moveDir == 2:
		goTo = (curr[0]+1, curr[1]) #South, i+1,j
	else: #moveDir == 3
		goTo = (curr[0], curr[1]+1) #East, i,j+1

	print("goTo", goTo)
	currOri, walkCount = calcTurns(goTo, curr, currOri) #turn
	for i in range(0, walkCount):
		walk()
		
		sensLeft = getSensorValue(3)
		sensRight = getSensorValue(5)
		sensFront = getSensorValue(4)
		print("sensLeft w", sensLeft)
		print("sensRight w", sensRight)
		print("sensFront w", sensFront)
		
		if sensLeft >= walkThresL:
			correctRight()
			
		elif sensRight >= walkThresR:
			correctLeft()
	
		elif sensFront >= walkThresF:
			print("BREAK")
			break
		
	r.sleep()
	
	count+=1
		
	visitedMap[goTo[0]][goTo[1]] = 1 #mark cell as visited
	
	print("visitedMap", visitedMap)
	print("count", count)
	
	if count < 64:
	#if count < 3:
		wander(myMap, visitedMap, goTo, currOri, count)
	
	return (curr, currOri)
	

	

# Main function
if __name__ == "__main__":
	rospy.init_node('example_node', anonymous=True)
	rospy.loginfo("Starting Group X Control Node...")
	
	# control loop running at 10hz
	r = rospy.Rate(3) # 10hz
	initializeRobot()
	setMotorTargetSpeed(1, 100)
	setMotorTargetSpeed(2, 100)
	setMotorTargetSpeed(3, 290)
	setMotorTargetSpeed(4, 290)
	setMotorTargetSpeed(9, 100)

	#for i in range(0,10):
		#sensLeft = getSensorValue(3)
		#rospy.loginfo("Left sensor value %d", sensLeft)
		#sensRight = getSensorValue(5)
		#rospy.loginfo("Right sensor value %d", sensRight)
		#sensFront = getSensorValue(4)
		#rospy.loginfo("Front sensor value %d", sensFront)
		


	import sys
	starti = int(sys.argv[1])
	startj = int(sys.argv[2])
	start = (starti, startj)

	endi = int(sys.argv[3])
	endj = int(sys.argv[4])
	end = (endi, endj)

	currOri = int(sys.argv[5])
	
	makeMap = int(sys.argv[6]) #1 if map blank
	
	
	myMap = EECSMap()
	#myMap.clearObstacleMap()
	myMap.printObstacleMap()
	if makeMap == 1 : #blank map, make it
		myMap.clearObstacleMap()
		visitedMap = [[0 for x in range(8)] for y in range(8)] #1 is visited
		visitedMap[0][0] = 1
		curr, currOri = wander(myMap, visitedMap, start, currOri, 1)
		
		myMap.printObstacleMap()	
		print(myMap.costMap)
		
		calcMapCost(end, myMap, start)
		#print("DONE")	
		print(myMap.costMap)
		#myMap.printObstacleMap()	
		currOri = navigateMap(myMap, curr, start, currOri) #go back to start
	
		navigateMap(myMap, start, end, currOri)
	
	else:
	
		myMap.printObstacleMap()	
		print(myMap.costMap)
		
		calcMapCost(end, myMap, start)
		#print("DONE")	
		print(myMap.costMap)
		#myMap.printObstacleMap()	
		#currOri = navigateMap(myMap, curr, start, currOri) #go back to start
	
		navigateMap(myMap, start, end, currOri)





    
    



