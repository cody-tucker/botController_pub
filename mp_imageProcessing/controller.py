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

# libraries for mqtt 
import paho.mqtt.client as mqttc
from queue import LifoQueue, Queue

import bot_module as bm                                     # custom module for bot and bobbin objs
print('[obj_server.py] bot_module successfully imported')

# ============================================
# MQTT functions
# ============================================

# callback for status channel
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
    #print("Connected with result code {}".format(rc))
    logging.debug("[controller.py] Connected flags = " + str(flags) + " rc= " + str(rc) +" ct_laptop")
    if rc == 0:
        client.connected_flag = True
        client.publish(TOPIC_4, "ct_laptop/connected", retain=True)
    else:
        client.bad_connection_flag = True

def on_disconnect(client, userdata, rc):
    print("Client was disconnected")

def on_message(client, userdata, message):
    print("Message recevied: "+message.payload.decode())

def on_log(client, userdata, level, buf):
    print(f'log: {buf}')

def on_message_botTaskList(client, userdata, message):
    #taskData = message.payload.decode("utf-8")
    taskData = message.payload.decode('utf-8').replace("'", '"').strip()               # decode data to string and replace ' with "
    json_taskData = json.loads(taskData)                                         # convert string BACK to dict
    botTaskStorageQ.put(json_taskData)
    #print(botTaskStorageQ)

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
        #bobbinsPerBotData = 0
    # elif 'hello' in statusData and bobbinsPerBotData == 2:
    #     client.publish(TOPIC_5, 0, qos=2, retain=False) # reset the task counter
    #     print('workaround resetting task counter.....')

def on_message_bobbinsPerBot(client, userdata, message):
    global bobbinsPerBotData 
    newBobbinsPerBotData = message.payload.decode("utf-8")
    newBobbinsPerBotData = newBobbinsPerBotData.split("/")
    newBobbinsPerBotData = int(newBobbinsPerBotData[-1])
    #bobbinsPerBotData += newBobbinsPerBotData

    bobbinsPerBotDataQ.put(newBobbinsPerBotData)
    if newBobbinsPerBotData == 1:
        bobbinsPerBotData += bobbinsPerBotDataQ.get()
    else:
        #bobbinsPerBotData -= (bobbinsPerBotDataQ.get() - 1)# after drop is performed 2 is sent out to reset variable
        bobbinsPerBotData = bobbinsPerBotDataQ.get()

    print(f'BOBBINS PER BOT RECEIVED= {bobbinsPerBotData}')
        

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

def on_message_botWaypoint(client, userdata, message):
    global waypointCounterData
    newWaypointCounter = message.payload.decode("utf-8")
    newWaypointCounter = newWaypointCounter.split("/")
    newWaypointCounter = int(newWaypointCounter[-1])

    waypointCounterDataQ.put(newWaypointCounter)
    #waypointCounterData = waypointCounterDataQ.get()

    if newWaypointCounter == 1:
        waypointCounterData += waypointCounterDataQ.get()
    elif newWaypointCounter == 0:
        waypointCounterData -= waypointCounterData # reset the waypoint counter data 
    else:
        pass

    print(f'WAYPOINT COUNTER = {waypointCounterData}')

def on_message_botTask(client, userdata, message):
    global taskCounterData
    newTaskCounter = message.payload.decode("utf-8")
    newTaskCounter = newTaskCounter.split("/")
    newTaskCounter = int(newTaskCounter[-1])

    taskCounterDataQ.put(newTaskCounter)

    if newTaskCounter == 1:
        taskCounterData += taskCounterDataQ.get()
    elif newTaskCounter == 0:
        taskCounterData -= taskCounterData # reset the task counter data
    else:
        pass

    print(f'TASK COUNTER = {taskCounterData}')
# ============================================


# ============================================
# Setup MQTT Variables
# ============================================
logging.debug('[controller.py] MQTT imported successfully')
#HOST = "192.168.178.82"                            # raspberrypi IP
HOST = "raspberrypi"
PORT = 1883
TOPIC_1 = "botInstruction"                          # topics to publish to
TOPIC_2 = "botRotation"                             # topics to publish to
TOPIC_3 = "botWaypoint"
TOPIC_4 = "connectionStatus"
TOPIC_5 = "botTask"
TOPIC_6 = "botDistance"
TOPIC_7 = "gripRightDistance"
TOPIC_8 = "gripLeftDistance"
# ============================================


# ============================================
# Define Controller Variables and Queue Objects
# ============================================
# waypoint_list = [[250,350], [500,350], [500,150], [350,200]]    # list of waypoints to move from current to target bobbin and avoid collisions... (x,y) for now... 
#                                                                 # in the future this would be generated via path planning algorithm....
# twist_dict = {6: ['temp', [500,375], [500,525]],
#               3: ['temp', [1000,375], [1000,525]]}  # bobbinID : (cur pos, waypoint, tar pos), temp gets reassigned based on bob ID in loop
twist_loc = [750, 450]                              # this now needs the point where twist should actually occur...

global twist_counter                                # workaround right now...
twist_counter = 0
bobbinsPerBotData = 0                               # initialize to 0
distanceToBobbinData = 0
statusQ = Queue()                                   # create queue object
bobbinsPerBotDataQ = Queue()                
distanceToBobbinDataQ = LifoQueue()                 # create LIFO queue so we always pull the most recent distance data
waypointCounterData = 0                             # initialize waypoint counter to start of list
waypointCounterDataQ = LifoQueue()
taskCounterData = 0
taskCounterDataQ = Queue()
gripRightDistanceDataQ = LifoQueue()
gripRightDistanceData = 0
gripLeftDistanceDataQ = LifoQueue()
gripLeftDistanceData = 0

botTaskStorageQ = Queue()

# ============================================

# ============================================
# Set Visualization Bools
# ============================================
GH_STATUS_VIS = True
global GH_PAUSE
GH_PAUSE = False
# ============================================

# ============================================
# Set UDP communication for streaming to GH
# ============================================
vis_port = 6001
vis_IP = socket.gethostbyname(socket.gethostname())
def gh_stream(IP, port, message):
    '''can only send strings'''
    vis_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vis_sock.sendto(bytes(message, "utf-8"),(IP,port))

# ============================================


####################
# SENDER SCRIPT 'Client'
####################
logging.basicConfig(level=logging.DEBUG)

# ============================================
# TCP Communication
# ============================================
server_port = 5007
controller_client_port = 5002
print('hi i am [controller.py]!')
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                       # create socket object and connect to server
s.bind((socket.gethostname(), controller_client_port))                      # bind this client to specific address
if s.connect_ex((socket.gethostname(), server_port)) == 0:
    logging.debug('[controller.py] connection successful')
else:
    logging.debug('[controller.py] connection failed...')
# ============================================

def main():
    twist_counter = 0
    # ============================================
    # Enable all MQTT Functions and Connect
    # ============================================
    client = mqttc.Client()                                                 # pretty sure this only works with the arduino pubsublibrary on MQTTv3
    client.enable_logger(logging.getLogger())                               # enable MQTT logging
    client.on_connect = on_connect
    client.will_set(TOPIC_4, "ct_laptop/disconnected", qos=0, retain=True)  # set what is sent on abnormal disconnect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    #client.on_log = on_log                                                 # set on log callback
    client.connect(HOST, PORT, keepalive=60)                                # connect to the pi server
    client.loop_start()                                                     # client keeps checking on subbed msgs
    logging.debug("[controller.py] MQTT loop has started...")
    client.subscribe("#")                                                   # sub to ALL topics
    client.message_callback_add("status", on_message_status)                # add callbacks ...
    client.message_callback_add("bobbinsPerBot", on_message_bobbinsPerBot)
    client.message_callback_add("distanceToBobbin", on_message_distanceToBobbin)
    client.message_callback_add("botWaypoint", on_message_botWaypoint)
    client.message_callback_add("botTask", on_message_botTask)
    client.message_callback_add("botTaskList", on_message_botTaskList)
    # ============================================


    # ============================================
    # Receive data from obj server and process
    # ============================================
    try:
        data = s.recv(2048)
    except:
        logging.debug('[controller.py] Cannot receive data...')
    while True:
        try:
            data = s.recv(2048)
        except:
            logging.debug('[controller.py] Cannot receive data...')
        if not data:
            break
        dict_str = data.decode('utf-8').replace("'", '"').strip()               # decode data to string and replace ' with "
        obj_data = json.loads(dict_str)                                         # convert string BACK to dict
        #print(f'CLIENT 2 received {obj_data}')
        #print(type(obj_data))
        #print(obj_data['4']['waypoints'])
        #print(len(obj_data['4']['tasks']))
    # ============================================
    # received dictionary structure is nested dict.

    # {'4': {'id': '4', 'type': 'bot', 'rotation': 251.0, 'location': [816, 464], 'estLocation': [814, 462],
    #  'rvec': [[-1.520025640866861, 2.194269921443196, -0.5971150714411827]],
    #  'tvevec': [[0.1554715128764604, 0.09111784192434641, 0.8412578371400234]], 'distances': [],
    #  'closestBobbin': [0, [0, 0], 0], 'targetAngle': 0, 'requiredRotation': 0,
    #  'waypoints': [[250, 350], [500, 3],50], [500, 150], [350, 200]], 'waypointDistance': 0,
    #  'tasks': {'6': ['temp', [500, 375], [500, 525]], '3': ['temp', [1000, 375], [1000, 525]]}
    #  'detectflag' : True}}

        # while not statusQ.empty():
        #     statusMessage = statusQ.get()

        #     if 'idle' in statusMessage or 'approach' in statusMessage:
        #         print(f'Bot is {statusMessage}. Bot has {bobbinsPerBotData} bobbin')
        #         print('*'*40)

        #         if taskCounterData < len(obj_data['4']['tasks']):
        #             #print(f'TaskCounterData = {taskCounterData}') 
        #             dict_pairs = list(obj_data['4']['tasks'].items())                                # convert to list
        #             currentTask = dict_pairs[taskCounterData]                               # pull current task from dict
        #             currentBobbin = currentTask[0]                                          # get the bobbin id from current task
        #             obj_data['4']['tasks'][currentBobbin][0] = obj_data[currentBobbin]['location']       # here assign the bobbins current location in the task waypoint dict
        #             obj_data['4']['waypoints'] = currentTask[1]
        #             print(obj_data['4']['waypoints'])

        #             client.publish(TOPIC_3, 1, qos=2, retain=False)             # add one to the waypoint counter
        #             client.publish(TOPIC_1, 3, qos=2, retain=False)
        #             client.publish(TOPIC_1, 1, qos=2, retain=False)
        #         else:
        #             pass

        #     else:
        #         pass


        # ============================================
        # Run controller loop
        # ============================================
        while not botTaskStorageQ.empty():
            taskMessage = botTaskStorageQ.get()
            print(type(taskMessage))
        while not statusQ.empty():
            statusMessage = statusQ.get()
            # ============================================
            # Send status to GH for visualization
            # ============================================
            if GH_STATUS_VIS:
                gh_stream(vis_IP, vis_port, str(statusMessage))
            if GH_PAUSE:
                pass


            # ============================================
            # check if IDLE. bot is awaiting instructions
            # ============================================
            if 'idle' in statusMessage or 'approach' in statusMessage:
                print(f'Bot is {statusMessage}. Bot has {bobbinsPerBotData} bobbin')
                print('Bot is awaiting instructions...')
                print('*'*40)

                # ============================================
                # check current task #. if !last item then it is a bobbin to pick up
                # ============================================
                if taskCounterData < len(obj_data['4']['tasks']) - 1:                                   # not the last item in the task dict
                    dict_pairs = list(obj_data['4']['tasks'].items())                                   # convert to list
                    currentTask = dict_pairs[taskCounterData]                                           # pull current task from dict
                    currentBobbin = currentTask[0]                                                      # get the bobbin id from current task
                    obj_data['4']['tasks'][currentBobbin][0] = obj_data[currentBobbin]['location']      # here assign the bobbins current location in the task waypoint dict
                    obj_data['4']['waypoints'] = currentTask[1]                                         # here assign the curent waypoints from the task dict with the new bobbin location
                    print(f'Bot is on task #{taskCounterData}')
                    #print(obj_data['4']['waypoints'])


                    # ============================================
                    # check waypoint counter. compute required rotation
                    # ============================================
                    if waypointCounterData < len(obj_data['4']['waypoints']):                                                                   # if we have not gone through all the waypoints...
                        print(f'current waypoint is {waypointCounterData}')
                        myRot = obj_data['4']['rotation']
                        print(f'current bot_rotation is {myRot}')
                        target_angle = bm.findAngleToWaypoint(obj_data['4']['estLocation'], obj_data['4']['waypoints'][waypointCounterData])       # compute angle to nearest waypoint
                        print(f'target_angle is {target_angle}')
                        required_rot = bm.computeRequiredRotation(obj_data['4']['rotation'], target_angle)                                      # compute how much to turn
                        print(f'required_rot is {required_rot}')


                        # ============================================
                        # check current rotation w/in tolerance
                        # ============================================
                        if abs(obj_data['4']['rotation'] - target_angle) <= 6:
                            print(f'YAY! Bot is within tolerance of target angle!')
                            waypoint_perimeter_distance = bm.findDistanceToWaypoint(obj_data['4']['estLocation'], obj_data['4']['waypoints'][waypointCounterData])
                            # distance_to_move = waypoint_perimeter_distance - 100

                            # ============================================
                            # check if within perimiter of the waypoint (pixels)
                            # ============================================
                            if waypoint_perimeter_distance < 150:
                                print(f'YAY! Bot is within waypoint perimeter')

                                # ============================================
                                # if first item in list then it is a bobbin to pickup, else it is a temporary waypoint
                                # ============================================
                                if waypointCounterData != 0:
                                    print('Bot is approaching a temporary waypoint...')
                                    if waypoint_perimeter_distance < 50:
                                        print('YAY, bot has reached the temp waypoint!')
                                        client.publish(TOPIC_3, 1, qos=2, retain=False)                 # add 1 to the waypoint counter
                                        client.publish(TOPIC_1, 1, qos=2, retain=False)                 # 1 = update to idle
                                    else:
                                        print(f'The bot is NOT at the temp waypoint')
                                        # print(f'Sending dist to bot...')
                                        # client.publish(TOPIC_2, required_rot, qos=2, retain=False)      # send distance to bot
                                        # print(f'Initiating moveForward(dist) command...')
                                        client.publish(TOPIC_1, 8, qos=2, retain=False)                 # 3 = move towards the bobbin.

                                else:
                                    # ============================================
                                    # check physical distance to bobbin from FRONT
                                    # ============================================
                                    client.publish(TOPIC_1, 6, qos=2, retain=False)                     # 6 = request FRONT distance from bot
                                    print('checking if bot is within pickup range...')
                                    if distanceToBobbinData < 8 and distanceToBobbinData > 0:
                                        print(f'YAY! The bot is within tolerable pick up distance!')
                                        if abs(obj_data['4']['rotation'] - target_angle) <= 5:              # added this to ensure it is closer, extra amount is subtracted from the alignRightGripper cmd
                                            print('Attemping to pickup bobbin...')
                                            if currentBobbin == '6':                    # grab with right
                                                client.publish(TOPIC_1, 13, qos=2, retain=False)                 # 4 = grab bobbin on instruction channel, arduino publishes updated bobbin count... 
                                                client.publish(TOPIC_3, 1, qos=2, retain=False)                 # add 1 to the waypoint counter
                                                client.publish(TOPIC_1, 1, qos=2, retain=False)                 # 1 = update to idle
                                            elif currentBobbin == '12':
                                                client.publish(TOPIC_1, 7, qos=2, retain=False)                 # drop right bobbin
                                            else: # grab with left
                                                client.publish(TOPIC_1, 4, qos=2, retain=False)                 # 4 = grab bobbin on instruction channel, arduino publishes updated bobbin count... 
                                                client.publish(TOPIC_3, 1, qos=2, retain=False)                 # add 1 to the waypoint counter
                                                #client.publish(TOPIC_1, 1, qos=2, retain=False)                 # 1 = update to idle
                                                client.publish(TOPIC_1, 5, qos=2, retain=False)                 # spin
                                                client.publish(TOPIC_1, 5, qos=2, retain=False)
                                                client.publish(TOPIC_1, 14, qos=2, retain=False)                # 14 drop left bobbin
                                                #time.sleep(1)
                                                client.publish(TOPIC_1, 1, qos=2, retain=False)                 # 1 = update to idle
                                            
                                            # time.sleep(2)
                                            # client.publish(TOPIC_1, 9, qos=2, retain=False)
                                            # if gripRightDistanceData < 5:
                                            #     print('YAY! Bobbin successfully grabbed!')
                                        else:
                                            print('Bot does not match target angle')
                                            client.publish(TOPIC_2, required_rot, qos=2, retain=False)                  # send required rotation angle to bot
                                            print('Initiating rotate command...')
                                            client.publish(TOPIC_1, 2, qos=2, retain=False)                             # 2 = rotate on instruction channel
                                            print('Bot is rotating...')

                                    else:
                                        print(f'The bot is NOT within tolerable pick up distance')
                                        client.publish(TOPIC_1, 3, qos=2, retain=False)                 # 3 = move towards the bobbin. maybe this should be a smaller increment than other movement...

                                    # ============================================
                                # ============================================

                            else:
                                print(f'Bot is not within waypoint perimeter')
                                print(f'waypoint_perimeter_distance = {waypoint_perimeter_distance}')       
                                if waypoint_perimeter_distance - 150 >= 90:
                                    print(f'distance for 8 = {waypoint_perimeter_distance - 150}')
                                    client.publish(TOPIC_1, 8, qos=2, retain=False)                         # 8 = move robot by large distance (15cm)
                                else:
                                    client.publish(TOPIC_1, 3, qos=2, retain=False)                         # 3 = move robot small distance (5cm)
                            # ============================================

                        else:
                            print('Bot does not match target angle')
                            if required_rot <= 10 and required_rot >= 0:                                 # attempting to overcome not rotating on really small values...
                                required_rot += 3
                            elif required_rot < 0 and required_rot >= -10:
                                required_rot -= 3
                            client.publish(TOPIC_2, required_rot, qos=2, retain=False)                  # send required rotation angle to bot
                            print('Initiating rotate command...')
                            client.publish(TOPIC_1, 2, qos=2, retain=False)                             # 2 = rotate on instruction channel
                            print('Bot is rotating...')
                        # ============================================


                    else:
                        print('YAY! all waypoints achieved. Task Complete!')
                        print('Resetting waypoint counter...')
                        client.publish(TOPIC_3, 0, qos=2, retain=False)                                 # send a 0 to reset the waypoint counter
                        print('Adding 1 to task counter')
                        client.publish(TOPIC_5, 1, qos=2, retain=False)                                 # add one to task counter
                        client.publish(TOPIC_1, 1, qos=2, retain=False)                                 # 1 = update to idle
                    # ============================================

                # ============================================
                # check current task #. if last item then it is spin location
                # ============================================
                elif taskCounterData == len(obj_data['4']['tasks']) - 1:                                # last item in the task dict
                    dict_pairs = list(obj_data['4']['tasks'].items())                                   # convert to list
                    currentTask = dict_pairs[taskCounterData]                                           # pull current task from dict
                    obj_data['4']['waypoints'] = currentTask[1]                                         # here assign the curent waypoints from the task dict with the new bobbin location
                    print(f'Bot is on task #{taskCounterData}')
                    #print(obj_data['4']['waypoints'])

                    # ============================================
                    # check waypoint counter. compute required rotation
                    # ============================================
                    if waypointCounterData < len(obj_data['4']['waypoints']):                                                                   # if we have not gone through all the waypoints...
                        print(f'current waypoint is {waypointCounterData}')
                        target_angle = bm.findAngleToWaypoint(obj_data['4']['location'], obj_data['4']['waypoints'][waypointCounterData])       # compute angle to nearest waypoint
                        print(f'target_angle is {target_angle}')
                        required_rot = bm.computeRequiredRotation(obj_data['4']['rotation'], target_angle)                                      # compute how much to turn
                        print(f'required_rot is {required_rot}')


                        # ============================================
                        # check current rotation w/in tolerance
                        # ============================================
                        if abs(obj_data['4']['rotation'] - target_angle) < 20:
                            print(f'YAY! Bot is within tolerance of target angle!')
                            waypoint_perimeter_distance = bm.findDistanceToWaypoint(obj_data['4']['location'], obj_data['4']['waypoints'][waypointCounterData])
                            

                            # ============================================
                            # check if within perimiter of the waypoint (pixels)
                            # ============================================
                            if waypoint_perimeter_distance < 100:
                                print(f'YAY! Bot is within waypoint perimeter')

                                # ============================================
                                # if it is the LAST item in list then it is SPIN location, else it is a temporary waypoint
                                # ============================================
                                if waypointCounterData < len(obj_data['4']['waypoints']) - 1:
                                    print('Bot is approaching a temporary waypoint...')
                                    if waypoint_perimeter_distance < 50:
                                        print('YAY, bot has reached the temp waypoint!')
                                        client.publish(TOPIC_3, 1, qos=2, retain=False)                 # add 1 to the waypoint counter
                                        client.publish(TOPIC_1, 1, qos=2, retain=False)                 # 1 = update to idle
                                    else:
                                        print(f'The bot is NOT at the temp waypoint')
                                        client.publish(TOPIC_1, 3, qos=2, retain=False)                 # 3 = move towards the bobbin.

                                else:
                                    # ============================================
                                    # approaching spin location
                                    # ============================================
                                    print('Bot is approaching spin location...')
                                    if waypoint_perimeter_distance < 35:
                                        print('YAY, bot has reached the spin waypoint!')
                                        client.publish(TOPIC_1, 5, qos=2, retain=False)                 # 5 = tell bot to twist
                                        print('Weeeeeeeeeeeeeeeee!')
                                        client.publish(TOPIC_3, 1, qos=2, retain=False)                 # add 1 to the waypoint counter
                                        client.publish(TOPIC_1, 1, qos=2, retain=False)                 # 1 = update to idle
                                    else:
                                        print(f'The bot is NOT at the spin waypoint')
                                        client.publish(TOPIC_1, 3, qos=2, retain=False)                 # 3 = move towards the bobbin.
                                    # ============================================
                                # ============================================

                            else:
                                print(f'Bot is not within waypoint perimeter')
                                client.publish(TOPIC_1, 3, qos=2, retain=False)                         # 3 = move towards the bobbin
                            # ============================================

                        else:
                            print('Bot does not match target angle')
                            client.publish(TOPIC_2, required_rot, qos=2, retain=False)                  # send required rotation angle to bot
                            print('Initiating rotate command...')
                            client.publish(TOPIC_1, 2, qos=2, retain=False)                             # 2 = rotate on instruction channel
                            print('Bot is rotating...')
                        # ============================================


                    else:
                        print('YAY! all waypoints achieved. Task Complete!')
                        print('Resetting waypoint counter...')
                        client.publish(TOPIC_3, 0, qos=2, retain=False)                                 # send a 0 to reset the waypoint counter
                        print('Adding 1 to task counter')
                        client.publish(TOPIC_5, 1, qos=2, retain=False)                                 # add one to task counter
                        client.publish(TOPIC_1, 1, qos=2, retain=False)                                 # 1 = update to idle
                    # ============================================



                # ============================================

                # ============================================
                # check current task #. all tasks completed. reset the task counter. make a unique status
                # ============================================
                else:                                                                                   # +1 more than the items in task dict (reset)
                    print(f'Bot is on task #{taskCounterData}')
                    print('Bot has completed all tasks....')
                    client.publish(TOPIC_1, 99, qos=2, retain=False)                                    # 99 = unique code for done with all tasks
                    print('Resetting task counter...')
                    client.publish(TOPIC_5, 0, qos=2, retain=False)                                     # reset the task counter
                # ============================================

            else:                                                                                       # bot is busy
                print(f'Bot is {statusMessage}. Bot has {bobbinsPerBotData} bobbin')
            # ============================================

        # ============================================






if __name__ == '__main__':
    main()