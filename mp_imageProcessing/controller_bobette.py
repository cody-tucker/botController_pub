"""robot controller script

This script recieves updated data from the object server about the bots and bobbins. 
It is connected to the MQTT network and provides instructions and feedback for the bot
Basically this is where the control loop goes and all MQTT stuff

"""

# libraries for tcp communication between scripts
import socket
import time
import logging
import json

# never again
import threading

# libraries for mqtt 
import paho.mqtt.client as mqttc
from queue import LifoQueue, Queue

import bot_module as bm                                     # custom module for bot and bobbin objs
print('[obj_server.py] bot_module successfully imported')

# ============================================
# MQTT functions
# ============================================
# options for bot status are:
# 1 - idle
# 2 - rotating
# 3 - approaching (5 cm) (ideally adaptive...)
# 4 - grab (right)
# 13 - grab (left)
# 5 - twisting
# 6 - request distance
# 7 - drop (right)
# 14 - drop (left)
# 8 - approaching (15 cm)
# 11 - request right gripper distance
# 12 - request left gripper distance

def on_connect(client, userdata, flags, rc):
    logging.debug("[controller.py] Connected flags = " + str(flags) + " rc= " + str(rc) +" ct_laptop")
    if rc == 0:
        client.connected_flag = True
        client.publish(CH_CONNECTIONSTATUS, f"ct_laptop/controller_{botname}/connected", retain=True)
    else:
        client.bad_connection_flag = True

def on_disconnect(client, userdata, rc):
    print("Client was disconnected")

def on_message(client, userdata, message):
    print("Message recevied: "+message.payload.decode())

def on_log(client, userdata, level, buf):
    print(f'log: {buf}')

def on_message_botTaskList(client, userdata, message):
    taskData = message.payload.decode('utf-8').replace("'", '"').strip()               # decode data to string and replace ' with "
    json_taskData = json.loads(taskData)                                               # convert string BACK to dict
    # check for which bot the task is for
    if json_taskData["bot"] == botname:
        botTaskStorageQ.put(json_taskData)

def on_message_coordinationFlags(client, userdata, message):
    coordData = message.payload.decode("utf-8")
    # check whos id is in the msg. If it is your name than you sent it. it is irrelevant
    # if it is the other bot name than store it.
    if len(coordData) < 7: # we would have to get to 4 digit task numbers until this will break...
        coordDataQ.put(coordData)
    else:
        coordDataQ.put('null')

def on_message_status(client, userdata, message):

    statusData = message.payload.decode("utf-8")    # decode incoming data to string
    statusQ.put(statusData)                         # place latest data in queue

    if 'idle' in statusData:
        print(f'The bot is {statusData}')
    elif 'rotating' in statusData:
        print(f'The bot is {statusData} to the target heading')
    elif 'approaching' in statusData:
        print(f'The bot is {statusData} the target')
    elif 'grabbing' in statusData:
        print(f'The bot is {statusData} the bobbin')
    elif 'twisting' in statusData:
        print(f'The bot is {statusData}. Weeeeee!')
    elif 'dropping' in statusData:
        print(f'The bot is {statusData} the bobbins')





def on_message_distanceToBobbin(client, userdata, message): 
    global distanceToBobbinData
    newDistanceToBobbinData = message.payload.decode("utf-8")
    newDistanceToBobbinData = newDistanceToBobbinData.split("/")
    newDistanceToBobbinData = int(newDistanceToBobbinData[-1])

    distanceToBobbinDataQ.put(newDistanceToBobbinData)
    distanceToBobbinData = distanceToBobbinDataQ.get()

    print(f'DISTANCE TO BOBBIN = {distanceToBobbinData} mm')

def on_message_gripRightDistance(client, userdata, message): 
    global gripRightDistanceData
    newgripRightDistanceData = message.payload.decode("utf-8")
    newgripRightDistanceData = newgripRightDistanceData.split("/")
    newgripRightDistanceData = int(newgripRightDistanceData[-1])

    gripRightDistanceDataQ.put(newgripRightDistanceData)
    gripRightDistanceData = gripRightDistanceDataQ.get()

    print(f'RIGHT GRIPPER DIST = {gripRightDistanceData} mm')

def on_message_gripLeftDistance(client, userdata, message): 
    global gripLeftDistanceData
    newgripLeftDistanceData = message.payload.decode("utf-8")
    newgripLeftDistanceData = newgripLeftDistanceData.split("/")
    newgripLeftDistanceData = int(newgripLeftDistanceData[-1])

    gripLeftDistanceDataQ.put(newgripLeftDistanceData)
    gripLeftDistanceData = gripLeftDistanceDataQ.get()

    print(f'LEFT GRIPPER DIST = {gripLeftDistanceData} mm')

def on_message_ghRotation(client, userdata, message): 
    global gh_rotationData
    
    newgh_rotationData = message.payload.decode("utf-8")
    newgh_rotationData = newgh_rotationData.split("/")
    newgh_rotationData = int(newgh_rotationData[-1])

    gh_rotationDataQ.put(newgh_rotationData)

def on_message_ghDistance(client, userdata, message): 
    global gh_distanceData
    
    newgh_distanceData = message.payload.decode("utf-8")
    newgh_distanceData = newgh_distanceData.split("/")
    newgh_distanceData = int(newgh_distanceData[-1])

    gh_distanceDataQ.put(newgh_distanceData)

def on_message_ghGoalPosition(client, userdata, message): 
    global gh_goalPosition
    
    newgh_goalPosition = message.payload.decode("utf-8")

    gh_goalPositionDataQ.put(newgh_goalPosition)

# ============================================
# ============================================
# Set Variables to Define Bot !!!!!!!!!!!!!!!!!!!
# IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# ============================================
GH_STATUS_VIS = True
global botname
botname = 'bobette'
global botID
botID = '13'
global bobID
bobID = '4'
subChannelList = [f'status/{botname}', 'distanceToBobbin', 'botTaskList', 'botCoordinationFlags/#',
                  f'gh/rotation/receive/{botname}', f'gh/distance/receive/{botname}', f'gh/bobbin/receive/{botname}']
checkBool = False # initialize checkBool
logging.basicConfig(level=logging.INFO)
# ============================================

# ============================================
# Setup MQTT Variables
# ============================================
logging.debug('[controller.py] MQTT imported successfully')
#HOST = "192.168.178.82"                            # raspberrypi IP
HOST = "raspberrypi"
PORT = 1883
CH_BOTINSTRUCTION = f'botInstruction/{botname}'     # topics to publish to
CH_TASKSTATUS = f'taskStatus/{botname}'
CH_COORDINATION = f'botCoordinationFlags/{botname}'
CH_CONNECTIONSTATUS = "connectionStatus"
CH_BOTROTATION = f'botRotation/{botname}'
CH_BOTDISTANCE = f'botDistance/{botname}'

CH_GH_ROTATION = f'gh/rotation/request/{botname}'
CH_GH_WAYPOINT = f'gh/waypoint/{botname}'
CH_GH_DISTANCE = f'gh/distance/request/{botname}'
CH_GH_BOBBIN = f'gh/bobbin/request/{botname}'

# current not used                         
TOPIC_6 = "botDistance"
TOPIC_7 = "gripRightDistance"
TOPIC_8 = "gripLeftDistance"
# ============================================

# ============================================
# Define Controller Variables and Queue Objects
# ============================================
distanceToBobbinData = 0
gripRightDistanceData = 0
gripLeftDistanceData = 0
gh_rotationData = 0
gh_distanceData = 0
gh_goalPosition = 0
statusQ = Queue()                                   # create queue object            
distanceToBobbinDataQ = LifoQueue()                 # create LIFO queue so we always pull the most recent distance data
gripRightDistanceDataQ = LifoQueue()
gripLeftDistanceDataQ = LifoQueue()
botTaskStorageQ = Queue(maxsize=0)
coordDataQ = Queue()
bot_dataQ = LifoQueue()

global gh_rotationDataQ
global gh_distanceDataQ
global gh_goalPositionDataQ
gh_rotationDataQ = LifoQueue()
gh_distanceDataQ = LifoQueue()
gh_goalPositionDataQ = LifoQueue()




# ============================================

# ============================================
# Set UDP communication for streaming to GH
# ============================================
vis_port = 6003
vis_IP = socket.gethostbyname(socket.gethostname())
def gh_stream(IP, port, message):
    '''can only send strings'''
    vis_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vis_sock.sendto(bytes(message, "utf-8"),(IP,port))
# ============================================


####################
# SENDER SCRIPT 'Client'
####################


# ============================================
# TCP Communication
# ============================================
server_port = 5007
controller_client_port = 5003
print(f'{botname}: hi i am [controller.py]!')
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                       # create socket object and connect to server
s.bind((socket.gethostname(), controller_client_port))                      # bind this client to specific address
if s.connect_ex((socket.gethostname(), server_port)) == 0:
    logging.debug(f'{botname}: [controller.py] connection successful')
else:
    logging.debug(f'{botname}: [controller.py] connection failed...')
# ============================================

# ============================================
# ultility functions
# ============================================
def sub(channel_list):
    for i in channel_list:
        client.subscribe(i)
    logging.debug(f'{botname}: [controller.py] succesfully connected to all channels')


def check_status():
    """
    check the latest status published by the robot
    """
    while not statusQ.empty():
        statusMessage = statusQ.get()
        if 'idle' in statusMessage or 'approach' in statusMessage:
            print(f'{botname}: is {statusMessage}.\nBot is awaiting instructions...\n'*'*40')
            return 'idle'
        else:
            print(f'{botname}: is {statusMessage}.')
            print('*'*40)

def check_flag(item):
    """
    Function checks the flagcheck property i.e. whether the task needs to wait for a specific signal before
    it is executed. Returns True or False. Input is the task dictionary
    """
    myTime = time.time()    
    print(item["flagcheck"])
    if item["flagcheck"] == 1:
        #print('here i am')
        if not coordDataQ.empty():
            check = coordDataQ.get()
            print(check)
        else:
            check = 'xx'
        while item["flag_id"] != check:
            client.publish(f'taskStatus/{botname}', f'FLAG CHECK NOT OKAY TO PROCEED... WAITING FOR {item["flag_id"]}')
            check = coordDataQ.get() # keep checking for the needed flag

        spentTime = time.time() - myTime
        client.publish(f'taskStatus/{botname}', f'FLAG CHECK OK TO PROCEED t={spentTime}', qos=2, retain=False)
        return True
    else:
        checkBool = True
        spentTime = time.time() - myTime
        client.publish(f'taskStatus/{botname}', f'FLAG CHECK NOT REQUIRED t={spentTime}', qos=2, retain=False)
        return True
# ============================================
# ============================================

# ============================================
# Task Reader class
# ============================================
class task_reader():
    def __init__(self, bot_id, taskItem):
        """
        Class to parse task information.
        """
        self.id = bot_id
        self.angleTolerance = 12                                # rotation tolerance for regular movement
        self.distance = 15                                      # the distance from a waypoint before it is completed
        self.waypointCounter = 0                                # +1 for each waypoint achieved
        self.goalCounter = 0                                    # +1 for each goal achieved (always 1 goal)
        self.goalDistance = 15                                  # the distance from the goal position before it is completed
        self.goalOrientationTolerance = 20                       # rotation tolerance at goal
        self.taskItem = taskItem                                # assign the task item dictionary (from GH) to the task reader
        self.task = self.taskItem["action"]
        self.flagID = self.taskItem["flag_id"]
        self.flagCheck = self.taskItem["flagcheck"]


    def check_task(self):
        """
        Function checks the assigned task type. Returns the string.
        """
        if self.task == 'grab':
            client.publish(CH_TASKSTATUS, 'TASK = GRAB', qos=2, retain=False)
            return self.task
        elif self.task == 'drop':
            client.publish(CH_TASKSTATUS, 'TASK = DROP', qos=2, retain=False)
            return self.task
        elif self.task == 'twist':
            client.publish(CH_TASKSTATUS, 'TASK = TWIST', qos=2, retain=False)
            return self.task
        else:
            client.publish(CH_TASKSTATUS, 'TASK = CROSS', qos=2, retain=False)
            return self.task



    def execute_waypoints(self, item, bot_data):
        """
        This function loops thru the waypoints to move to the FINAL WAYPOINT.
        Returns True if arrived at the final waypoint.
        """

        # i = 0
        # while i < 4:
        #     print(f'moving to waypoint {i}')
        #     i += 1
        #     time.sleep(1.0)
        # return True
            
            
        myWaypoints = item["waypoints"]
        #curBobId = item["bobbins"][0]
        #curBobbin = bot_data[curBobId]["location"]

        # if the task does not have waypoints, this will be an empty string ''
        if not myWaypoints:
            return True

        while self.waypointCounter < len(myWaypoints):
            bot_data = bot_dataQ.get()                  # always checking the most recent Queue item
            print(f'{botname}: my current waypoint is : {myWaypoints[self.waypointCounter]}')

            # SEND THE CURRENT WAYPOINT TO GH
            client.publish(CH_GH_WAYPOINT, f'{myWaypoints[self.waypointCounter]}', qos=2, retain=False)
            time.sleep(0.5)
            client.publish(CH_GH_DISTANCE, 1, qos=2, retain=False) # send a 1 to request distance compute from GH

            checkDistanceToWaypoint = self.check_waypoint_perimeter(bot_data=bot_data, move=False, distance=self.distance, goal=False)
            time.sleep(2.0)
            if not checkDistanceToWaypoint:

                checkRot = self.check_rotation(bot_data=bot_data, goal=False)
                time.sleep(2.0)
                if checkRot:
                    #print('now we are here...')
                    checkWaypoint = self.check_waypoint_perimeter(bot_data=bot_data, move=True, distance=self.distance, goal=False)

                else:
                    pass
            else: 
                self.waypointCounter += 1
                print(f'{botname}: myWaypointCounter = {self.waypointCounter}')
        return True
            
            
    def approach_goal(self, item, bot_data):
        """
        This function moves to the goal position and checks the orientation upon arrival.
        Returns True if arrived at the goal position and is in the goal orientation.
        for drop it goes to the bobbin goal pos
        for grab it goes to the current location of the bobbin
        """

        # i = 0
        # while i < 1:
        #     print(f'approaching goal {i}')
        #     i += 1
        #     time.sleep(1.0)
        # return True


        curBobId = item["bobbins"][0]
        #curBobbinPos = bot_data[curBobId]["location"]
        #goalOrientation = item["bobbingoalorient"]

        if self.task != 'grab':
            goalPosition = item["goalpos"]
            myGoal = goalPosition
            # some tasks will not have a goal position (twist, cross)
            if not goalPosition:
                return True
        else:
            goalPosition = []
            client.publish(CH_GH_BOBBIN, f'1,{curBobId}', qos=2, retain=False)          # send the current bobbin id to GH
            time.sleep(0.5)
            client.publish(CH_GH_BOBBIN, f'0,{curBobId}', qos=2, retain=False)
            # in GH this should trigger sending it thru to the waypoint channel
            print(f'{botname}: goal position requested from GH')
            time.sleep(1.0)
            myGoal = gh_goalPositionDataQ.get()
            goalPosition.append(myGoal)

        print(f'{botname}: goal position = {goalPosition}')

        #goalPosition = item["goalpos"]

        while self.goalCounter < 1:
            bot_data = bot_dataQ.get()
            
            # SEND THE CURRENT WAYPOINT TO GH
            client.publish(CH_GH_WAYPOINT, f'{myGoal}', qos=2, retain=False)
            time.sleep(0.5)
            client.publish(CH_GH_DISTANCE, 1, qos=2, retain=False) # send a 1 to request distance compute from GH

            checkDistanceToWaypoint = self.check_waypoint_perimeter(bot_data=bot_data, move=False, distance=self.goalDistance, goal=True)
            time.sleep(2.0)
            if not checkDistanceToWaypoint:

                checkRot = self.check_rotation(bot_data=bot_data, goal=True)
                time.sleep(2.0)
                if checkRot:
                    checkWaypoint = self.check_waypoint_perimeter(bot_data=bot_data, move=True, distance=self.goalDistance, goal=True)
                else:
                    pass
            else:
                self.goalCounter += 1
                print(f'{botname}: myGoalCounter = {self.goalCounter}')
        return True


    def check_rotation(self, bot_data, goal):
        """
        This function computes the bot current rotation and checks if it is within tolerance.
        Returns True/False
        """
        #global gh_rotationDataList
        gh_rotationDataList = []
        # NEW ROTATION FUNCTION. ASKS GRASSHOPPER FOR ROTATION.
        # FIND CURRENT ROTATION
        client.publish(CH_GH_ROTATION, 1, qos=2, retain=False) # send a 1 to request required rotation compute from GH
        time.sleep(0.1)
        #client.publish(CH_GH_ROTATION, 0, qos=2, retain=False) # send a 0 to tell GH to stop sending rotation.

        statusData = statusQ.get()
        if 'idle' in statusData:

            
            #time.sleep(0.2)
            req_rotation = gh_rotationDataQ.get()
            gh_rotationDataList.append(req_rotation)
            print(f'{botname}: length of rot list = {len(gh_rotationDataList)}')
            if len(gh_rotationDataList) > 1:
                gh_rotationDataList = gh_rotationDataList[-1]
            print(f'{botname}: rot received = {gh_rotationDataList}')

            # print("checking rotation")
            # myRot = bot_data[self.id]['rotation']
            # myRot = -myRot # needs to be negative to work with GH
            # print(f'myRot = {myRot}')
            print(f'{botname}: required rotation is {req_rotation}')

            if abs(req_rotation) < self.angleTolerance and goal == False:
                print(f'{botname}: YAY! Bot is within tolerance of target angle!')
                return True
            if abs(req_rotation) < self.goalOrientationTolerance and goal == True:
                print(f'{botname}: YAY! Bot is within tolerance of target angle!')
                return True
            else:
                #req_rotation = gh_rotationDataQ.get()
                #gh_rotationDataList.append(req_rotation)
                req_rotation = gh_rotationDataList.pop(0) # pop the first item 
                print(f'{botname}: req rotation = {req_rotation}')
                print(f'{botname}: publishing rotation...')
                client.publish(CH_BOTROTATION, req_rotation, qos=2, retain=False)
                print(f'{botname}: Bot is NOT within tolerance of target angle...')
                time.sleep(1.0)
                client.publish(CH_BOTINSTRUCTION, 2, qos=2, retain=False)               # 2 is rotate
                print(f'{botname}: bot should now execute rotation')
                #time.sleep(1.0)
                return False                          

    def check_waypoint_perimeter(self, bot_data, move, distance, goal):
        """
        Check if bot is within a specified distance of the waypoint.
        Returns True/False
        """
        #global gh_distanceDataList
        gh_distanceDataList = []

        #statusData = statusQ.get()
        #if 'idle' in statusData:

        # NEW CHECK DISTANCE FUNCTION FOR WAYPOINT
        # FIND CURRENT DISTANCE FROM GH
        client.publish(CH_GH_DISTANCE, 1, qos=2, retain=False) # send a 1 to request distance compute from GH
        #time.sleep(0.1)
        #client.publish(CH_GH_DISTANCE, 0, qos=2, retain=False) # send a 0 to tell GH to stop sending distance.
        time.sleep(0.5)
        my_distance = gh_distanceDataQ.get()
        gh_distanceDataList.append(my_distance)
        print(f'{botname}: distance = {gh_distanceDataList}')

        print(f"{botname}: checking distance to waypoint")
        #waypoint_perimeter_distance = bm.findDistanceToWaypoint(bot_data[self.id]['camLocation'], waypointList[waypointCounter])
        if my_distance <= distance:
            print(f'{botname}: YAY! Bot is within waypoint perimeter')
            return True
        elif my_distance > distance and move == True and goal == False:
            print(f'{botname}: Bot is NOT within waypoint perimeter...I NEED TO MOVE')
            #my_distance = gh_distanceDataQ.get()
            my_distance = gh_distanceDataList.pop(0)
            #print(my_distance)
            print(f'{botname}: publishing distance to move')
            # send distance to botDistance
            if abs(my_distance) < 5:
                client.publish(CH_BOTDISTANCE, 6, qos=2, retain=False)
            else:
                client.publish(CH_BOTDISTANCE, f'{abs(my_distance)}', qos=2, retain=False)
            time.sleep(1.0)
            client.publish(CH_BOTINSTRUCTION, 3, qos=2, retain=False)
            print(f'{botname}: bot should now move {abs(my_distance)}')
            return False
        elif my_distance > distance and move == True and goal == True:
            print(f'{botname}: Bot is NOT within waypoint perimeter...I NEED TO MOVE')
            #my_distance = gh_distanceDataQ.get()
            my_distance = gh_distanceDataList.pop(0)
            #print(my_distance)
            print(f'{botname}: publishing distance to move')
            # send distance to botDistance
            if abs(my_distance-18) < 5:
                client.publish(CH_BOTDISTANCE, 5, qos=2, retain=False)
            else:
                client.publish(CH_BOTDISTANCE, f'{abs(my_distance-18)}', qos=2, retain=False)
            time.sleep(1.0)
            client.publish(CH_BOTINSTRUCTION, 3, qos=2, retain=False)
            print(f'{botname}: bot should now move {abs(my_distance-18)}')
            return False
        else:
            print(f'{botname}: Bot is NOT within waypoint perimeter...')
            return False

    def execute_task(self, item, bot_data):
        """
        This function publishes the instuction to activate the low-level action. Either grab, drop, twist, or cross.
        it also performs check_gripper for grab / drop
        """
        print(f'{botname}: performing task')
        bot_data = bot_dataQ.get()

        if self.task == 'grab':
            gripper = self.check_gripper(item)
            if gripper == 'right':
                client.publish(CH_BOTINSTRUCTION, 4, qos=2, retain=False)               # 4 is right gripper
                # verify pickup?
                time.sleep(4.0)
                #client.publish(f'debug/{botname}', 'g', qos=2, retain=False)
                #client.publish(CH_BOTINSTRUCTION, 1, qos=2, retain=False)               # 1 = update to idle
                return True
            else:
                client.publish(CH_BOTINSTRUCTION, 13, qos=2, retain=False)              # left gripper
                # verify pickup?
                time.sleep(4.0)
                #client.publish(f'debug/{botname}', 'g', qos=2, retain=False)
                #client.publish(CH_BOTINSTRUCTION, 1, qos=2, retain=False)               # 1 = update to idle
                return True

        elif self.task == 'drop':
            gripper = self.check_gripper(item)
            print(f'{botname}: I AM GONNA DROP THE BOBBIN!!!!')
            time.sleep(15)
            if gripper == 'right':
                client.publish(CH_BOTINSTRUCTION, 7, qos=2, retain=False) # 7 is right gripper
                return True
            else:
                client.publish(CH_BOTINSTRUCTION, 14, qos=2, retain=False)
                return True

        elif self.task == 'twist':
            twistAngle = item["angle"]
            print(f'{botname}: publishing rotation for twist...')
            client.publish(CH_BOTROTATION, twistAngle, qos=2, retain=False)
            time.sleep(1.0)
            client.publish(CH_BOTINSTRUCTION, 2, qos=2, retain=False) # send command to rotate
            return True

        elif self.task == 'cross':
            # check which bot is on the left = cross 1 (15). right = cross 2 (16)
            # check bot data and compare pixels
            bob_pos_X = bot_data[botID]['location'][0]
            bobette_pos_X = bot_data[bobID]['location'][0]

            myRot = 0 - bot_data[botID]['rotation']
            crossRot = -270
            if abs(crossRot - myRot) > 20:
                if crossRot - myRot <= 0:
                    reqRot = crossRot - myRot
                    client.publish(CH_BOTROTATION, reqRot, qos=2, retain=False)
                    time.sleep(0.2)
                    client.publish(CH_BOTINSTRUCTION, 2, qos=2, retain=False)
                else:
                    reqRot = (crossRot - myRot) - 360
                    client.publish(CH_BOTROTATION, reqRot, qos=2, retain=False)
                    time.sleep(0.2)
                    client.publish(CH_BOTINSTRUCTION, 2, qos=2, retain=False)

            time.sleep(2.0)

            if bob_pos_X < bobette_pos_X:
                print(f'{botname} is on the left')
                client.publish(CH_BOTINSTRUCTION, 15, qos=2, retain=False)
                return True
                #time.sleep(1.0)
            else:
                print(f'{botname} is on the right')
                client.publish(CH_BOTINSTRUCTION, 16, qos=2, retain=False)
                return True


    def check_gripper(self, item):
        """
        Function to check which gripper is assigned to drop or pickup
        """
        gripper = item["gripper"]
        print(f'{botname}: the gripper is {gripper}')
        return gripper
# ============================================

# ============================================
# Main controller
# ============================================
def worker(lock):

    while True:
        #with lock:
        # try to make the queue objects here...


        while not botTaskStorageQ.empty():



            item = botTaskStorageQ.get()                                                        # get the next task from the queue
            x_data = bot_dataQ.get()                                                            # get the most recent data from obj_server
            myTaskReader = task_reader(bot_id=botID, taskItem=item)                             # initialize task_reader for current task
            logging.debug(f'[controller_{botname}.py] Task Reader initialized')
            client.publish(f'botInstruction/{botname}', 1, qos=2, retain=False)                 # start the loop by setting BOT status to idle
            #client.publish(f'status/{botname}', 'idle', qos=2, retain=False)                    # start the loop by MANUALLY setting BOT status to idle
            print(f'{botname}: Working on task #{item["task_id"]}')


            checkBool = check_flag(item=item)                                               # check if other robots have completed necessary previous tasks   
            if checkBool:                                                                       # the loop is based on the task type
                
                if myTaskReader.task == 'grab':
                    client.publish(CH_TASKSTATUS, 'executing GRAB', qos=2, retain=False)
                elif myTaskReader.task == 'drop':
                     client.publish(CH_TASKSTATUS, 'executing DROP', qos=2, retain=False)
                elif myTaskReader.task == 'twist':
                     client.publish(CH_TASKSTATUS, 'executing TWIST', qos=2, retain=False)
                elif myTaskReader.task == 'cross':
                     client.publish(CH_TASKSTATUS, 'executing CROSS', qos=2, retain=False)
                    
                waypointsCompleted = myTaskReader.execute_waypoints(item, x_data)           # move thru the assigned waypoints
                if waypointsCompleted:                                    
                    print(f'{botname}: waypoints finished')                        

                    goalCompleted = myTaskReader.approach_goal(item, x_data)                # move to the goal position (bobbin or other + orientation)
                    if goalCompleted:
                        print(f'{botname}: goal position achieved')

                        taskCompleted = myTaskReader.execute_task(item, x_data)                     # perform low-level action (grab, drop, twist, cross)
                        time.sleep(2.0)
                        if taskCompleted:
                            print(f'{botname}: {myTaskReader.task} has been executed')

                            if item["flagpub"] == 1:                                        # check if you need to publish notification of completion
                                client.publish(CH_COORDINATION, f'{botname}{item["task_id"]}')
                                client.publish(CH_BOTINSTRUCTION, 1, qos=2, retain=False)   # 1 = update to idle
                            else:
                                client.publish(CH_BOTINSTRUCTION, 1, qos=2, retain=False)   # 1 = update to idle

                        else:
                            print(f'{botname}: {myTaskReader.task} was not successfully executed....')
                    else:
                        print(f'{botname}: bot is not at the goal position yet...')
                else:
                    print(f'{botname}: waypoints not completed')

                # elif myTaskReader.task == 'drop':
                #     client.publish(CH_TASKSTATUS, 'executing DROP', qos=2, retain=False)
                #     if item["flagcheck"] == 1:
                #         client.publish(CH_COORDINATION, f'{botname}{item["task_id"]}')
                #     else:
                #         pass


                # elif myTaskReader.task == 'twist':
                #     client.publish(CH_TASKSTATUS, 'TWIST', qos=2, retain=False)
                #     if item["flagcheck"] == 1:
                #         pass
                #     else:
                #         pass    
                

                # elif myTaskReader.task == 'cross':
                #     client.publish(CH_TASKSTATUS, 'CROSS', qos=2, retain=False)
                #     if item["flagcheck"] == 1:
                #         pass
                #     else:
                #         pass


            client.publish(CH_TASKSTATUS, f'task {item["task_id"]} done', qos=2, retain=False)
            print(f'{botname}: Finished task #{item["task_id"]}')
            print(f'{botname}: Resetting checkBool')
            checkBool = False
            myTaskReader.waypointCounter = 0            # reset variables before closing task
            myTaskReader.goalCounter = 0
            gh_rotationDataList = []
            gh_distanceDataList = []

            print(f'{botname}: Resetting gh queues')
            gh_goalPositionDataQ = LifoQueue()
            gh_distanceDataQ = LifoQueue()
            gh_rotationDataQ = LifoQueue()
            gh_rotationDataQ.queue.clear()
            gh_distanceDataQ.queue.clear()
            gh_goalPositionDataQ.queue.clear()

            client.publish(CH_GH_DISTANCE, 0, qos=2, retain=False) 
            client.publish(CH_GH_ROTATION, 0, qos=2, retain=False)


            botTaskStorageQ.task_done()
            botTaskStorageQ.join()



        # print(f'{botname} waiting for next task...')
        # gh_rotationDataQ.mutex.acquire()
        # gh_rotationDataQ.queue.clear()
        # gh_rotationDataQ.all_tasks_done.notify_all()
        # gh_rotationDataQ.unfinished_tasks = 0
        # gh_rotationDataQ.mutex.release()
        # gh_rotationDataQ = LifoQueue()
        # gh_distanceDataQ = LifoQueue()
        # gh_goalPositionDataQ = LifoQueue()
        # print(gh_rotationDataQ.empty())
# ============================================

# ============================================
# Receive data from obj server and process
# ============================================
def read_data(lock):
    global obj_data
    try:
        data = s.recv(2048)
    except:
        logging.debug(f'{botname}: [controller.py] Cannot receive data...')
    while True:
        try:
            data = s.recv(2048)
        except:
            logging.debug(f'{botname}: [controller.py] Cannot receive data...')
        if not data:
            break
        dict_str = data.decode('utf-8').replace("'", '"').strip()               # decode data to string and replace ' with "
        obj_data = json.loads(dict_str)                                         # have to set a time.sleep in obj_server and image_client to avoid sending too much and unable to parse (still a bit buggy tho...)
        bot_dataQ.put(obj_data)                                                 # put data in Queue to share with other thread
# ============================================


def main():
    lock = threading.Lock()                                                 # lock for thread 

    # ============================================
    # Enable all MQTT Functions and Connect
    # ============================================
    global client
    client = mqttc.Client()                                                 # pretty sure this only works with the arduino pubsublibrary on MQTTv3
    client.enable_logger(logging.getLogger())                               # enable MQTT logging
    client.on_connect = on_connect
    client.will_set(CH_CONNECTIONSTATUS, f'{botname}: ct_laptop/disconnected', qos=0, retain=True)  # set what is sent on abnormal disconnect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    #client.on_log = on_log                                                 # set on log callback
    client.connect(HOST, PORT, keepalive=60)                                # connect to the pi server
    client.loop_start()                                                     # client keeps checking on subbed msgs
    logging.debug(f'{botname}: [controller.py] MQTT loop has started...')
    #client.subscribe("#")                                                  # sub to topics
    sub(subChannelList)
    client.message_callback_add(f"status/{botname}", on_message_status)                # add callbacks ...
    #client.message_callback_add("distanceToBobbin", on_message_distanceToBobbin)
    client.message_callback_add("botTaskList", on_message_botTaskList)
    client.message_callback_add("botCoordinationFlags/#", on_message_coordinationFlags)
    client.message_callback_add(f'gh/rotation/receive/{botname}', on_message_ghRotation)
    client.message_callback_add(f'gh/distance/receive/{botname}', on_message_ghDistance)
    client.message_callback_add(f'gh/bobbin/receive/{botname}', on_message_ghGoalPosition)
    # ============================================
    

    
    # ============================================
    # Create threads
    # ============================================
    taskParserThread = threading.Thread(target=worker, args=(lock,), daemon=None)         # start the main thread
    readDataThread = threading.Thread(target=read_data, args=(lock,), daemon=None)        # start the thread to receive bot obj data
    readDataThread.start()
    taskParserThread.start()
    botTaskStorageQ.join()                                                                # close queue when tasks done
    bot_dataQ.join()

    # ============================================



if __name__ == '__main__':
    main()