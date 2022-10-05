"""obj management script

This script manages bot and bobbin objects. It receives data from image processing and assigns it to the
correct objects.

A separate script will run that is the 'control flow' for the robot actions. That script will request data
from here based on the current action via MQTT.

"""

# https://www.positronx.io/create-socket-server-with-multiple-clients-in-python/


from os import name
import socket
import time
import logging
import json
import numpy as np
from _thread import *
from copy import deepcopy

import bot_module as bm                                     # custom module for bot and bobbin objs
print('[obj_server.py] bot_module successfully imported')


# ============================================
# Set UDP communication for streaming to GH
# ============================================
vis_port = 6002
vis_IP = socket.gethostbyname(socket.gethostname())
def gh_stream(IP, port, message):
    '''can only send strings'''
    vis_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vis_sock.sendto(bytes(message, "utf-8"),(IP,port))

# ============================================


def multi_threaded_client(connection, addr_dict):
    '''
    function to spawn a new thread for each client connected to the server. arguements are
    the connection obj returned from socket.accept() and the dictionary containing all connected
    socket objects and their address, key = port #. ex: addr_dict = {port#: [socket obj, (addr tuple)]}

    inside here is where bot objects are updated and data is fowarded to controller script
    '''
    connection.send(str.encode('Server is working:'))
    while True:
        mySender = socket.socket.getpeername(connection)                # get the address of connected obj
        try:
            data = connection.recv(2048)                                # get data
        except:
            logging.debug(f'[obj_server.py] Cannot receive data from {mySender[1]}')
            break
        if mySender[1] == 5001:                                         # if this thread connection is port 5001
            if not data:                                                # aka if the data is from image client
                break                                                   # then we process it accordingly
            # ============================================
            # Assign to objects here
            # ============================================
            # received dictionary structure
            # obj_data =  # {ID# : ID#, (detected position), 'null', (estimated position), rvecs, tvecs}
            myTime = round(time.time(), ndigits=2)                          # get current time stamp

            dict_str = data.decode('utf-8').replace("'", '"').strip()               # decode data to string and replace ' with "
            obj_data = json.loads(dict_str)                                 # convert string BACK to dict
            #json_data = json.dumps(dict_data, indent=4, sort_keys=True)    # convert to JSON format (maybe if needed)

            # for all the incoming data, assign it to the correct bot or bobbin obj
            for x in obj_data.keys():
                if x in botID_list: 
                    bots[x].location = obj_data[x][1]
                    #bots[x].estLocation = obj_data[x][3]
                    bots[x].rvec = np.asarray(obj_data[x][4])               # needs to be an array for the Rodriguez conversion
                    bots[x].tvec = np.asarray(obj_data[x][5])
                    bots[x].camLocation = obj_data[x][6]
                    bots[x].computeBotRotation()

                    #bots[x].waypoints = [[250,350], [500,350], [500,150], [350,200]] # dont think i need this anymore...
                    #bots[x].tasks = {'6': ['temp', [500,375]], '3': ['temp', [1000,375]], 'spin': [[1000,525],[500,525]]}
                    #bots[x].tasks = {'3': ['temp'], '6': ['temp'], '1': ['temp'], 'spin': [[800,525],[650,525]]} # this should come from the task parser.
                    # either all at once, in chunks, or able to change in real time from GH/Rhino. 
                    # more importantly, this needs to be unique to each Bot... check if the bot is ID x, then assign that task list?

                    #bots[x].detectflag
                    #print(bots[x].rotation)
                else:
                    bobbins[x].location = obj_data[x][1]
                    #bobbins[x].estLocation = obj_data[x][3]
                    bobbins[x].rvec = np.asarray(obj_data[x][4])            # needs to be an array for the Rodriguez conversion
                    bobbins[x].tvec = np.asarray(obj_data[x][5])
                    bobbins[x].camLocation = obj_data[x][6]
                    #bobbins[x].computeBotRotation()
                    #print(bobbins[x].rotation)
            # ============================================

            # ============================================
            # convert bot objects to dicts and format for JSON  
            # ============================================                          # JSON is very particular about what characters are allowed etc
            for x in bots.keys():                                                   # especially if u will use json.loads to build a dict from bytes
                bot_data[x] = bots[x].__dict__                                      # convert all bot objects into a dict!!! 
                for y in bot_data[x].keys():        
                    if type(bot_data[x][y]) is np.ndarray:                          # check if the item is type np array
                        bot_data[x][y] = bot_data[x][y].tolist()                    # cannot send np.array as JSON
                    elif type(bot_data[x][y]) is tuple:                             # LOL cant have parenthesis when rebuilding
                        bot_data[x][y] = list(bot_data[x][y])                       # dict from JSON... omg
            # ============================================

            # NOW WE ONLY NEED THE BOT DATA IN CONTROLLER
            # JUST SEND THIS INFO. OTHER DATA GOES TO GH
            for x in bots.keys():                                                   # especially if u will use json.loads to build a dict from bytes
                bot_dict[x] = bots[x].__dict__
                for y in bot_dict[x].keys():        
                    if type(bot_dict[x][y]) is np.ndarray:                          # check if the item is type np array
                        bot_dict[x][y] = bot_dict[x][y].tolist()
                    elif type(bot_dict[x][y]) is tuple:                             # LOL cant have parenthesis when rebuilding
                        bot_dict[x][y] = list(bot_dict[x][y])

            # ============================================
            # add bobbin objects to the same dict...  
            # ============================================
            for x in bobbins.keys():
                bot_data[x] = bobbins[x].__dict__
                for y in bot_data[x].keys():        
                    if type(bot_data[x][y]) is np.ndarray:                          
                        bot_data[x][y] = bot_data[x][y].tolist()                    
                    elif type(bot_data[x][y]) is tuple:                             
                        bot_data[x][y] = list(bot_data[x][y])                       
            # ============================================

            if len(addr_dict) > 1 and len(bot_data) > 0:                            # this is to make sure we have connected to both clients
                try:                                                                # before sending data
                    addr_dict[5002][0].sendall(bytes(f'{bot_dict}', 'utf-8'))       # send dict as a bytes encoded string to controller
                    addr_dict[5003][0].sendall(bytes(f'{bot_dict}', 'utf-8'))       # send dict as a bytes encoded string to controller_copy
                    #print(bytes(f'{bot_dict}', 'utf-8'))
                    gh_stream(vis_IP, vis_port,str(bot_data))                       # send data to gh as well via udp
                    #time.sleep(0.1)
                except:
                    logging.debug('[obj_server.py] Cannot send data...')
        else:
            if not data:
                break
            print(f'msg from {mySender}')                                           # data was sent from the controller 5002
            
    connection.close()
    logging.debug('Quitting [obj_server.py] from [image_client.py]')




####################
# 'Server'
# build bot and bobbin objects based on known ID's and count
# receive data from image processing stream and assign to correct bot/bobbin objects
# then send that data to the approproate controller
####################
logging.basicConfig(level=logging.DEBUG)
# ============================================
# TCP communication
# ============================================
server_port = 5007                                                  # set stable server port
addr_dict = {}                                                      # dict to store all connected clients 
print('hi i am [obj_server.py]!')
# create socket object and connect
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)               # SOCK_STREAM is for tcp
try:
    s.bind((socket.gethostname(), server_port))                     # bind server to specified port
except socket.error as e:
    logging.debug(f'[obj_server.py] Connection error is {e}')
logging.debug('[obj_server.py] Waiting for connection')
logging.debug('[obj_server.py] Socket is listening...')
s.listen(4)                                                         # set to accept 3 connection (always n+1)
# ============================================


# ============================================
# Initializing bot and bobbin objects
# ============================================
botID_list = [4, 13]
bobbinID_list = [1,2,5,6]
bot_data = {}
bot_dict = {}

botID_list = bm.listOfIntToListOfStrings(botID_list)                # convert to strings because the key needs to be
bobbinID_list = bm.listOfIntToListOfStrings(bobbinID_list)          # sent as a string for json.loads

bots = {}                                                           # dict to store bot objects
bobbins = {}                                                        # dict to store bobbin objects

for i, bot in enumerate(botID_list):                                # initialize bots
    bots[bot] = bm.bot(bot)

for i, bobbin in enumerate(bobbinID_list):                          # initialize bobbins
    bobbins[bobbin] = bm.bobbin(bobbin)
# ============================================



# ============================================
# Receive Data from Image Processing and Spawn Threads for each client
# Inside the Threads assign image processing data to bot and bobbin objects (see start_new_thread fn)
# ============================================
def main():
    thread_count = 0
    while True:
        conn, addr = s.accept()                                             # conn is the socket obj created on connection, addr is the address of connection
        addr_dict[addr[1]] = [conn, addr]                                   # add the socket obj and address to dict of connected clients
        logging.debug(f'[obj_server.py] successfully connected to {addr}') 
        start_new_thread(multi_threaded_client, (conn, addr_dict, ))        # start a new thread for each client
        thread_count += 1                                                   # update connection / thread counter
        print('Thread Number: ' + str(thread_count))
        print('*'*40)
        
    #conn.close()
    #logging.debug('Quitting [obj_server.py] from [image_client.py]') 
# ============================================

if __name__ == '__main__':
    main()