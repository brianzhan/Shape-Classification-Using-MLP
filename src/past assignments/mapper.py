#!/usr/bin/env python
import roslib
import rospy
import math
from fw_wrapper.srv import *
from map import *
import time
import sys
import pickle
import signal
from copy import copy

"""
Obstacle Map: 
 --- --- --- --- --- --- --- --- 
| O   O   O   O   O   O   O   O | 
                                 
| O | O   O   O   O   O   O   O | 
                                 
| O   O   O   O   O   O   O   O | 
     --- ---                     
| O   O   O | O   O   O   O   O | 
 --- ---                         
| O   O   O | O   O   O   O   O | 
                                 
| O | O | O   O | O   O   O   O | 
         ---                     
| O | O   O | O   O   O   O   O | 
     --- ---                     
| O   O   O   O | O   O   O   O | 
 --- --- --- --- --- --- --- --- 
Cost Map:
1 0 0 0 0 0 0 0  
1 0 0 0 0 0 0 0  
1 0 0 0 0 0 0 0  
1 1 1 0 0 0 0 0  
0 0 1 0 0 0 0 0  
1 0 1 1 0 0 0 0  
1 0 0 1 0 0 0 0  
1 1 1 1 0 0 0 0  

"""

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


def signal_handler(sig, frame):
    #stop()
    StopAllWheels()
    print "signal handler called"
    newmap.printObstacleMap
    location = raw_input('File name = ')
    pickle.dump(newmap, open(location,'wb'))
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

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
def stop():
    wheels = (11,12,13,14)
    value = (0,0,0,0)
    set_multi_wheel_speed(wheels, value)


def setMotorsMove():
    setMotorMode(11, 1)
    setMotorMode(12, 1)
    setMotorMode(13, 1)
    setMotorMode(14, 1)


def StopAllWheels():
    motor_ids = (11, 12, 13, 14)
    target_values = (1024, 1024, 1024, 1024)
    set_multiple_wheel_torque(motor_ids, target_values)

def DirectionLocator(block): # Note: Block must be an  int of 1,2,3,4
    global theta, xPosn, yPosn
    theta = int(theta)
    if (theta is 0):
        yPosn += block
    elif (theta is 1):
        xPosn += block
    elif (theta is 2):
        yPosn -= block
    elif (theta is 3):
        xPosn -= block
    

def MoveForwardBlock(block):
    global theta
    option = 1 #0 is no proportional control, 1 is with
    if option is 0:
        setMotorsMove()
    #2048 is stop, 812 is fastest,
        motor_ids = (11, 12, 13, 14)
        target_values = (2047, 1023, 1023, 2047)
        setMotorTargetPositionCommand(10, 514) #look front
        rospy.sleep(1)
        set_multiple_wheel_torque(motor_ids, target_values)
        rospy.sleep(1.65)
        StopAllWheels()
        DirectionLocator(block)
    else:
        MoveTryFollow()

def TurnRight():
    global theta
    tspeed = 600
    setMotorsMove()
    motor_ids = (11, 12, 13, 14)
    target_values = (tspeed, tspeed, tspeed, tspeed)    
    set_multiple_wheel_torque(motor_ids, target_values)
    rospy.sleep(1.5)    
    StopAllWheels()
    theta = (theta + 1) % 4

def TurnLeft():
    global theta
    
    tspeed = 1615
    setMotorsMove()
    motor_ids = (11, 12, 13, 14)
    target_values = (tspeed, tspeed, tspeed, tspeed)    
    set_multiple_wheel_torque(motor_ids, target_values)
    rospy.sleep(1.45)
    StopAllWheels()
    theta = (theta - 1) % 4

def TurnAround():
    TurnLeft()
    TurnLeft()

def MoveTryFollow():
    dmsPort = 1
    dmsblocked = 1300
    setMotorTargetPositionCommand(10, 800) #look left
    rospy.sleep(1)
    dmsdist = getSensorValue(dmsPort)
    print "Prior dmsdist was", dmsdist
    if dmsdist > dmsblocked : #check if left is blocked
        print("WallFollowLeft")
        WallFollowLeft()
    else: #check if right is blocked
        setMotorTargetPositionCommand(10, 224) #look right
        rospy.sleep(1.5)
        dmsdist = getSensorValue(dmsPort)
        print "Current dmsdist is ", dmsdist 
        if dmsdist > dmsblocked:
            print("WallFollowRight")
            WallFollowRight()
        else:
            setMotorsMove()
            #2048 is stop, 812 is fastest,
            motor_ids = (11, 12, 13, 14)
            target_values = (2047, 1020, 1020, 2047)    
            set_multiple_wheel_torque(motor_ids, target_values)
            setMotorTargetPositionCommand(10, 514) #look right
            rospy.sleep(1)
            starttime = time.time()
            while time.time() - starttime < 1.65:
                print time.time() - starttime
                dmsdist = getSensorValue(1)
                if dmsdist > 1400:
                    break
            StopAllWheels()
            DirectionLocator(1)
    #AdjustFront()

def WallFollowLeft():
    global theta
    dmsPort = 1
    leftIR = 4
    rightIR = 5
    setMotorsMove()
    # dmsvalue = getSensorValue(1)
    irvalue = getSensorValue(leftIR)
    # if dmsvalue > 1760 and dmsvalue < 2000 and irvalue < 100 and irvalue > 30:
    #     print "IR AND DMS ARE CLOSE TO WALL with DMS as ", dmsvalue, "and IR as ", irvalue 
    #     tspeed = 600
    #     setMotorsMove()
    #     motor_ids = (11, 12, 13, 14)
    #     target_values = (tspeed, tspeed, tspeed, tspeed)    
    #     set_multiple_wheel_torque(motor_ids, target_values)
    #     rospy.sleep(0.5)  
    # else:
    #     AdjustAngle("left")

    # rospy.sleep(1)
    dmsvalue = getSensorValue(1)
    rawerror = math.exp((dmsvalue - 4491.68)/(-1223.778)) #centimeters
    rawedist = rawerror - 10
    ykp = 27 #from centimeters to turn
    xkp = 27
    timekp = 0.05
    if(rawedist > 0):
        right = 0
        left = ykp*rawedist
    else: #turn right
        right = (xkp*rawedist)*-1
        left = 0

    timee = timekp*abs(rawedist) + 1.65
    motor_ids = (11, 12, 13, 14)
    target_values = (2047-right, 1023-left, 1023-left, 2047-right)
    setMotorTargetPositionCommand(10, 514)
    rospy.sleep(1)
    set_multiple_wheel_torque(motor_ids, target_values)
    starttime = time.time()
    while time.time() - starttime < 1.7:
        dmsdist = getSensorValue(1)
        if dmsdist > 1400:
            break
    #forward is 1024-2048, move 11 slower, so turn right # move slower in the counterclockwise direction
    StopAllWheels()
    DirectionLocator(1)
    print 2047-right, " ", 1023-left

def WallFollowRight():
    global theta
    dmsLookRight = 224
    dmsPort = 1
    setMotorsMove()
    leftIR = 4
    rightIR = 5
    setMotorsMove()
    # dmsvalue = getSensorValue(1)
    irvalue = getSensorValue(leftIR)
    # if dmsvalue > 1760 and dmsvalue < 2000 and irvalue < 100 and irvalue > 30:
    #     print "IR AND DMS ARE CLOSE TO WALL with DMS as ", dmsvalue, "and IR as ", irvalue        
    #     tspeed = 1615
    #     setMotorsMove()
    #     motor_ids = (11, 12, 13, 14)
    #     target_values = (tspeed, tspeed, tspeed, tspeed)    
    #     set_multiple_wheel_torque(motor_ids, target_values)
    #     rospy.sleep(0.5)  
    # else:
    #     AdjustAngle("right")
    # rospy.sleep(1)

    dmsvalue = getSensorValue(1)
    rawerror = math.exp((dmsvalue - 4491.68)/(-1223.778)) #centimeters
    rawedist = rawerror - 10

    ykp = 27 #from centimeters to turn
    xkp = 27
    timekp = 0.05

    if(rawedist > 0):
        right = ykp*rawedist
        left = 0
    else: #turn right
        right = 0
        left = (xkp*rawedist)*-1

    timee = timekp*abs(rawedist) + 1.65
    motor_ids = (11, 12, 13, 14)
    target_values = (2047-right, 1023-left, 1023-left, 2047-right)
    setMotorTargetPositionCommand(10, 514) #look right
    rospy.sleep(1)
    set_multiple_wheel_torque(motor_ids, target_values)
    starttime = time.time()
    
    while time.time() - starttime < 1.7:
        dmsdist = getSensorValue(1)
        if dmsdist > 1400:
            break

    StopAllWheels()
    DirectionLocator(1)
    print 2047-right, " ", 1023-left

def AdjustAngle(dir):
    dmsvalue = getSensorValue(1)
    rawerror = math.exp((dmsvalue - 4491.68)/(-1223.778)) #centimeters
    rawedist = rawerror - 10

    while(abs(rawedist) > 4.7):    
        dmsvalue = getSensorValue(1)
        rawerror = math.exp((dmsvalue - 4491.68)/(-1223.778)) #centimeters
        rawedist = rawerror - 10
        if dir is "right":
            if (rawedist > 0):
                tspeed = 100
                setMotorsMove()
                motor_ids = (11, 12, 13, 14)
                target_values = (tspeed, tspeed, tspeed, tspeed)    
                set_multiple_wheel_torque(motor_ids, target_values)

            else: #turn right
                tspeed = 1125
                setMotorsMove()
                motor_ids = (11, 12, 13, 14)
                target_values = (tspeed, tspeed, tspeed, tspeed)    
                set_multiple_wheel_torque(motor_ids, target_values)

        elif dir is "left":
            if (rawedist > 0):
                tspeed = 1125
                setMotorsMove()
                motor_ids = (11, 12, 13, 14)
                target_values = (tspeed, tspeed, tspeed, tspeed)    
                set_multiple_wheel_torque(motor_ids, target_values)

            else: #turn right
                tspeed = 100
                setMotorsMove()
                motor_ids = (11, 12, 13, 14)
                target_values = (tspeed, tspeed, tspeed, tspeed)    
                set_multiple_wheel_torque(motor_ids, target_values)

    StopAllWheels()
    
# def AdjustFront():
#     #lower = 1000
#     #ideal = 2700
#     #anything higher is blind

#     dmsblocked = 1200
#     setMotorTargetPositionCommand(10, 514) #look front
#     rospy.sleep(1.5)
#     dmsdist = getSensorValue(1)
#     if dmsdist < 2800 and dmsdist > 2400:
#         print "dms dist is ", dmsdist
#         print "stop all wheels called"
#         StopAllWheels()
#     elif dmsdist > dmsblocked:
#         print dmsdist
#         motor_ids = (11, 12, 13, 14)
#         target_values = (2047, 1023, 1023, 2047)
#         set_multiple_wheel_torque(motor_ids, target_values)
#         rospy.sleep(0.01)
#         StopAllWheels()
#         AdjustFront()

def goSouth():
    global theta
    theta = int(theta)
    print "goSouth with theta as", theta
    if  theta == 0: #north
        print("TurnAround()")
        TurnAround()
    elif theta is 3: #west
        print("TurnLeft()")
        TurnLeft()
    elif theta is 1: #east
        print("TurnRight()")
        TurnRight()

    print("MoveForwardBlock(1)")
    MoveForwardBlock(1)

def goEast():
    global theta
    theta = int(theta)
    print "goEast with theta as", theta
    if  theta == 0: #north
        print("TurnRight()")
        TurnRight()
    elif theta is 3: #west
        print("TurnAround()")
        TurnAround()
    elif theta is 2: #south
        print("TurnLeft()")
        TurnLeft()
    print("MoveForwardBlock(1)")
    MoveForwardBlock(1)


def goWest():
    global theta
    theta = int(theta)
    print "goWest with theta as", theta
    print theta
    if  theta == 0: #north
        print("TurnLeft()")
        TurnLeft()
    elif theta == 1: #east
        print("TurnAround()")
        TurnAround()
    elif theta is 2: #south
        print("TurnRight()")
        TurnRight()
    print("MoveForwardBlock(1)")
    MoveForwardBlock(1)

def goNorth():
    global theta
    theta = int(theta)
    print "goNorth with theta as", theta
    if  theta == 2: #south
        print("TurnAround()")
        TurnAround()
    elif theta is 3: #west
        print("TurnRight()")
        TurnRight()
    elif theta is 1: #east
        print("TurnLeft()")
        TurnLeft()
    print("MoveForwardBlock(1)")
    MoveForwardBlock(1)

# ******************************************************************************************
# --------------------------------Start of Movement Planning---------------------------------------------

def PathFindTerminal():
#     global newmap
    # mapFileName = raw_input("Provide a name for map = ")
    # newmap = pickle.load(open(mapFileName, 'rb'))
    # newmap.printObstacleMap()
    start_x, start_y = raw_input("Starting point [x] [y] = ").split()
    end_x, end_y = raw_input("Ending point [x] [y]=").split()
    heading = raw_input("Direction (North 0, East 1, South 2, West 3) = ")
    start_x = int(start_x)
    start_y = int(start_y)
    end_x = int(end_x)
    end_y = int(end_y)
    theta = heading
    start = [start_x, start_y]
    end = [end_x, end_y]

def signal(signal, frame):
    saveLocation = raw_input("Saving file. Enter file name")
    pickle.dump(newmap,open(saveLocation, 'wb'))
    StopAllWheels()
    sys.exit(0)

# used exclusively for SetWalls
def theta_dir(theta):
    if theta == 0:
        return DIRECTION.North
    if theta == 1:
        return DIRECTION.East
    if theta == 2:
        return DIRECTION.South
    if theta == 3:
        return DIRECTION.West

    
# 1 denotes FR, 2 denotes FL, 3 denotes C (LR), 4 denotes FRL blocked, 5 denotes empty
def SetWalls(newmap, i, j):
    global theta
    dmsBlocked = 1200
    IRBlocked = 60
    # left = raw_input("left blocked (0 no) (1 yes) = ")
    # right = raw_input("right blocked (0 no) (1 yes) = ")
    # front = raw_input("front blocked (0 no) (1 yes) =  ")
    # left = int(left)
    # right = int(right)
    # front = int(front)\
    setMotorTargetPositionCommand(10,512)
    rospy.sleep(1)
    RobotActionDebug(theta)
    # if front is 1:
    #     newmap.setObstacle(i,j,1,theta_dir(theta))
    # if right is 1:
    #     newmap.setObstacle(i,j,1,theta_dir((theta + 1) % 4))
    # if left is 1:
    #     newmap.setObstacle(i,j,1,theta_dir((theta - 1) % 4))
    if (getSensorValue(frontDMS) > dmsBlocked):
        newmap.setObstacle(i,j,1,theta_dir(theta))
    if (getSensorValue(rightIR) > IRBlocked ):
        newmap.setObstacle(i,j,1,theta_dir((theta + 1) % 4))
    if (getSensorValue(leftIR) > IRBlocked):
        newmap.setObstacle(i,j,1,theta_dir((theta - 1) % 4))



def RobotActionDebug(theta):
    dmsBlocked = 1400
    IRBlocked = 60
    if (getSensorValue(frontDMS) > dmsBlocked):
        print "front open; ",
    else:
        print "front blocked; ",
    if (getSensorValue(leftIR) > IRBlocked):
        print "left open;  ",
    else:
        print "left blocked; ",
    if (getSensorValue(rightIR) > IRBlocked):
        print "right open; ",
    else:
        print "right blocked; ",
    if (theta is 0):
        print "Facing North \n"
    if (theta is 1):
        print "Facing East \n"
    if (theta is 2):
        print "Facing South \n"
    if (theta is 3):
        print "Facing West \n"


#return list of adjacent directions
def getAdjacent(mymap, i, j):
    queue = []
    if mymap.getNeighborObstacle(i, j, DIRECTION.North) == 0 and (i-1) > -1:
        queue = queue + [[i-1, j]]
    if mymap.getNeighborObstacle(i, j, DIRECTION.South) == 0 and (i+1) < 8:
        queue = queue + [[i+1, j]]
    if mymap.getNeighborObstacle(i, j, DIRECTION.East) == 0 and (j+1) < 8:
        queue = queue + [[i, j+1]]
    if mymap.getNeighborObstacle(i, j, DIRECTION.West) == 0 and (j-1) > -1:
        queue = queue + [[i, j-1]]
    return queue


#this function takes in a map and the end position and sets the cost for each cell next to it
def setcostmap(mymap, pos):
    cost = mymap.getCost(pos[0], pos[1])
    adjacent = getAdjacent(mymap, pos[0], pos[1])
    for node in adjacent:
        cell = mymap.getCost(node[0], node[1]) #get the cost of first open adjacent
        if cell > cost or cell == 0: #if the adjacent cell is 0 or has a higher cost
            mymap.setCost(node[0], node[1], cost + 1) #set the adjacent cell to new cost
            setcostmap(mymap, [node[0], node[1]]) #run recursively on new cell
    return


#assume that the costmap is already assigned
def wavefront(mymap, start, path):
    #print(start)
    cost = mymap.getCost(start[0], start[1])
    if cost == 1:
        path += [start]
        return path
    else:
        adjacent = getAdjacent(mymap, start[0], start[1])
        for a in adjacent:
            if mymap.getCost(a[0], a[1]) == (cost - 1):
                path += [a]
                wavefront(mymap, path[-1], path)
                return path
        return "no path"

def searchpath(mymap, start, goal):
    mymap.clearCostMap()
    mymap.setCost(goal[0], goal[1], 1)
    setcostmap(mymap, goal)
    path = wavefront(mymap, start, [])
    return path

def getAdjacentFrontTrue(start, unvisited):
    global theta
    for node in unvisited:
        if theta is 0 and unvisited.count([start[0]-1, start[1]])!=0:
            return True
        elif theta is 1 and unvisited.count([start[0], start[1]+1])!=0:
            return True
        elif theta is 2 and unvisited.count([start[0]+1, start[1]])!=0:
            return True
        elif theta is 3 and unvisited.count([start[0], start[1]-1])!=0:
            return True
    return False

def ExploreMap(newmap, i, j):
    global unvisited
    i = int(i)
    j = int(j)
    #set current cell to visited: visited = 1, unvisited = 0   
    newmap.setCost(i, j, 1)

    #Sense Blocked Walls
    SetWalls(newmap, i, j)
    newmap.printObstacleMap()
    newmap.printCostMap()
    #Get Adjacent Open Cells
    adj = getAdjacent(newmap, i, j)
    #compare open cells to ones that we've already visited and add into unvisited
    for neighbor in adj:
        if newmap.getCost(neighbor[0], neighbor[1]) != 1 and neighbor not in unvisited: # if not visited, append
            unvisited.append(neighbor)
    #if unvisited is empty, you are done
    if not unvisited:
        return newmap
    searchmap = EECSMap()
    # use path finder to explore these neighbors
    searchmap.horizontalWalls = newmap.horizontalWalls
    searchmap.verticalWalls = newmap.verticalWalls
    print "unvisited nodes are", unvisited
    print "Current direction is (0=N, 1=E, 2=S, 3=W)", theta
    start = [i,j]
    emptyFrontTrue = getAdjacentFrontTrue(start, unvisited) # @Type: Boolean
    if (emptyFrontTrue is True):
        if (theta is 0): # Go north
            nextPosn = [i-1, j]
            i-=1
        elif (theta is 1): # Go east
            nextPosn = [i, j+1]
            j+=1
        elif (theta is 2): # Go south
            nextPosn = [i+1, j]
            i+=1
        elif (theta is 3): # Go west
            nextPosn = [i, j-1]
            j-=1
        MoveForwardBlock(1)
        print "MoveForward(1) directly"
        unvisited.remove([i,j])
        goal = [i,j] # Technically we just moved here
        print "check unvisited", unvisited
    else:
        goal = unvisited.pop()
        print "Goal is", goal
        gothroughmaze(start, searchpath(searchmap, start, goal))
    RobotNewPositionDebug(goal, theta)
    #recursively call ExploreMap
    ExploreMap(newmap, goal[0], goal[1])

def RobotNewPositionDebug(start, theta):
    print "Robot is now in position ", start, " ",
    if (theta is 0):
        print "Facing North"
    if (theta is 1):
        print "Facing East"
    if (theta is 2):
        print "Facing South"
    if (theta is 3):
        print "Facing West"

#start is []
#path is list of []'s
def gothroughmaze(start, path):

    global theta, xPosn, yPosn
    start[0] = int(start[0])
    start[1] = int(start[1])


    xPosn = start[0]
    yPosn = start[1]

    next = path[0]
    if xPosn is next[0] and yPosn is next[1]:
        return "At goal!"

    xdif = start[0] - next[0]
    ydif = start[1] - next[1]

    if ydif is -1:
        goEast()
    if ydif is 1:
        goWest()
    if xdif is -1:
        goSouth()
    if xdif is 1:
        goNorth()
    path.pop(0)
    gothroughmaze(next, path)
    
def DmsLookForward():
    setMotorTargetPositionCommand(10, 512) # DMS is at motor 10, test will this be called?

# Global variables:
frontDMS = 1
leftIR = 4
rightIR = 5
cm15Analog = 1355 # DMS analog number
dmsLookLeft =  800# AX-12A Motor Position Value
dmsLookRight = 224 # AX-12A Motor Position Value
xPosn = 0
yPosn = 0
leftIRBlocked = 60
rightIRBlocked = 60
global theta
global unvisited

# Main function
if __name__ == "__main__":
    mymap = EECSMap()
    newmap = EECSMap()
    
    rospy.init_node('example_node', anonymous=True)

    rospy.loginfo("Starting Group L Control Node...")
    setMotorMode(11, 0)

    r = rospy.Rate(10) # 10hz
    StopAllWheels()
    rospy.sleep(1)
    setMotorTargetPositionCommand(10, 512)
    #theta = 2
    # while not rospy.is_shutdown():
    #     #setMotorTargetPositionCommand(10, 812) #look right
    #     print(getSensorValue(leftIR), " ", getSensorValue(1))
    #     theta = 2
    #     #setMotorTargetPositionCommand(10, 224) #look right
    #     #WallFollowRight()
    #     #TurnLeft()
    #     #MoveTryFollow()
    # r.sleep()
    #AdjustFront()

    #1 is no walls
    #TurnRight()
    StopAllWheels()
    mode = raw_input("Enter mode (Explore 1) or (Path 2)) ")
    mode = int(mode)

    if mode is 1:
        theta = 2
        unvisited = []
        ExploreMap(newmap,0,0)
    elif mode is 2:
        # PathFindTerminal()

        start_x, start_y = raw_input("Starting point [x] [y] = ").split()
        end_x, end_y = raw_input("Ending point [x] [y]=").split()
        heading = raw_input("Direction (North 0, East 1, South 2, West 3) = ")

        start_x = int(start_x)
        start_y = int(start_y)
        end_x = int(end_x)
        end_y = int(end_y)

        theta = heading
        start = [start_x, start_y]
        end = [end_x, end_y]

        gothroughmaze(start,searchpath(mymap, start, end)) 
        print(searchpath(mymap,start,end))
        mymap.printCostMap()
        mymap.printObstacleMap()
    else:
        print("Wrong Input")

    # r.sleep()
    # stop()
