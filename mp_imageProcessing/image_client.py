"""image processing script

This script processes image data, identifies aruco markers, tracks them,
and sends the data to the obj_server.

"""

# https://stackoverflow.com/questions/40059654/python-convert-a-bytes-array-into-json-format/40060181


# libraries for tcp communication between scripts
import socket
import time
import logging

# libraries for image processing
import numpy as np
import cv2
import pyrealsense2 as rs
from pykalman import KalmanFilter
from copy import deepcopy

####################
# SENDER SCRIPT 'Client'
####################
logging.basicConfig(level=logging.DEBUG)

# ============================================
# TCP Communication
# ============================================
server_port = 5007
image_client_port = 5001
print('hi i am [image_client.py]!')
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)               # create socket object and connect to server
s.bind((socket.gethostname(), image_client_port))                   # bind this client to specific address
if s.connect_ex((socket.gethostname(), server_port)) == 0:
    logging.debug('[image_client.py] connection successful')
else:
    logging.debug('[image_client.py] connection failed...')
# ============================================

# ============================================
# Visualization Settings
# ============================================
CUBE_VIS = False
WAYPOINT_VIS = False
# ============================================


# ============================================
# Camera Setup
# ============================================
#camera matrix from factory intrinsics 640 x 480
color_cam_matrix_640x480 = np.zeros((3,3), dtype=np.float32)
# [ 606.344      0       325.859]
# [ 0         606.099    243.574]
# [ 0            0          0   ]
color_cam_matrix_640x480.itemset((0,0), 606.344)
color_cam_matrix_640x480.itemset((0,2), 325.859)
color_cam_matrix_640x480.itemset((1,1), 606.099)
color_cam_matrix_640x480.itemset((1,2), 243.574)
color_cam_matrix_640x480.itemset((2,2), 1)


#camera matrix from factory intrinsics 1280 x 720
color_cam_matrix_1280x720 = np.zeros((3,3), dtype=np.float32)
# [ 909.515      0       648.788]
# [ 0         909.148    365.361]
# [ 0            0          0   ]
color_cam_matrix_1280x720.itemset((0,0), 909.515)
color_cam_matrix_1280x720.itemset((0,2), 648.788)
color_cam_matrix_1280x720.itemset((1,1), 909.148)
color_cam_matrix_1280x720.itemset((1,2), 365.574)
color_cam_matrix_1280x720.itemset((2,2), 1)

#camera matrix from factory intrinsics 1920 x 1080
color_cam_matrix_1920x1080 = np.zeros((3,3), dtype=np.float32)
# [ 1364.27      0       973.182]
# [ 0         1363.72    548.042]
# [ 0            0          0   ]
color_cam_matrix_1920x1080.itemset((0,0), 1364.27)
color_cam_matrix_1920x1080.itemset((0,2), 973.182)
color_cam_matrix_1920x1080.itemset((1,1), 1363.72)
color_cam_matrix_1920x1080.itemset((1,2), 548.042)
color_cam_matrix_1920x1080.itemset((2,2), 1)

# assign matrix for resolution
color_cam_matrix = color_cam_matrix_1280x720
print(color_cam_matrix)

# establish camera pipeline
pipeline = rs.pipeline()
# create config object
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

# start streaming
profile = pipeline.start(config)
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# aruco dict
#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

# Check realsense communication
logging.debug('[image_client.py] Testing for camera data connection...')
try:
	np.asanyarray(pipeline.wait_for_frames().get_color_frame().get_data())
except:
	raise Exception("[image_client.py] Can't get rgb frame from camera data source")
# ============================================



# ============================================
# Setup KF 
# ============================================
# Kalman variables (see notes)
initial_state_mean = np.asarray([0,0,0,0])
#print(np.shape(initial_state_mean))

transition_matrix = [[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]

observation_matrix = [[1, 0, 0, 0],
                      [0, 0, 1, 0]]

transistionCov=1.0e-1*np.eye(4)
observationCov=1.0e-4*np.eye(2)
#transistionCov=1.0e-2*np.eye(4)      #OG   # you can play with the size of number to change how much
#observationCov=1.0e-1*np.eye(2)      #OG   # either affects the prediction. make transition higher = more reliance on observation and vis versa
#observationCov = np.ones((2,2))

myKF = KalmanFilter(transition_matrices= transition_matrix,
                   observation_matrices= observation_matrix,
                   initial_state_mean= initial_state_mean,
                   #initial_state_covariance= initcovariance,
                   transition_covariance= transistionCov,
                   observation_covariance= observationCov,
                   em_vars=['transition covariance', 'initial_state_covariance', 'observation_covariance'])
# ============================================

id_dict = {} # {ID# : ID#, (detected position), [list of last detected positions], (estimated position),
             #  rvecs, tvecs}

#id_dict = {'12' : [12, np.asarray((0, 0)), [], 0, [], []]}
#obj_dict =  # {ID# : ID#, (detected position), 'null', (estimated position), rvecs, tvecs}

print('[image_client.py] Press "q" to exit')

# ============================================
# IMAGE PROCESSING 
# ============================================
def main():
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # here we get factory camera intrinsic parameters
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        color_dist_coeffs = color_intrin.coeffs
        color_dist_coeffs = np.asarray(color_dist_coeffs)

        # =======================================
        # ADDED FOR DEPTH
        # =======================================
        # Aligning depth stream to color stream
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        # Include to receive XYZ points
        # This gets the camera intrinsic properties such as focal length, distortion coeff etc.
        # Needed for  deprojection (2D to 3D)
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        #print(depth_intrin)
        # =======================================

        # Read frames from capture
        _, frame = cap.read()
        rgb_vis = np.asanyarray(color_frame.get_data())


        # =======================================
        # ADDED FOR DEPTH
        # =======================================
        # Convert depth_frame for visualizing in cv2
        depth_vis = np.asanyarray(depth_frame.get_data())
        # if images are too dark, try normalizing based on an ROI rather than the entire image
        depth_vis = cv2.normalize(depth_vis, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        #depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        # =======================================

        # ARUCO DETECTION
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejects = cv2.aruco.detectMarkers(gray, aruco_dict)

        # DRAW waypoints
        if WAYPOINT_VIS:
            #bot_tasks = {'6': ['temp', [500,500]], '3': ['temp', [900,375]], 'spin': [[800,525],[650,525]]}
            #bot_tasks = {'6': ['temp'], '3': ['temp'], 'spin': [[800,525],[650,525]]}
            bot_tasks = {'6': ['temp'], '3': ['temp'], '12': [[800,525]], 'spin': [[800,525],[650,525]]}
            for j, x in enumerate(bot_tasks.keys()):
                if x == 'spin':
                    for i, num in enumerate(bot_tasks[x]):
                        if i == 0:
                            cv2.circle(frame, (num[0], num[1]), 8, (120,0,200), -1)
                            cv2.putText(frame, f'spin waypoint #{i}', (num[0] - 70, num[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120,0,200), 1)
                        else:
                            cv2.circle(frame, (num[0], num[1]), 8, (0,0,255), -1)
                            cv2.putText(frame, f'spin here!!', (num[0] - 50, num[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                else:
                    for i, num in enumerate(bot_tasks[x]):
                        if type(num) is list:
                            cv2.circle(frame, (num[0], num[1]), 8, (120,0,200), -1)
                            cv2.putText(frame, f'waypoint #{j}-{i}', (num[0] - 70, num[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120,0,200), 1)


        # ============================================
        # DETECT MARKERS 
        # THESE ARE BASIC OPERATIONS THAT PROVIDE CONTROLLER WITH INFORMATION ABOUT THE BOTS AND BOBBINS AND THEIR PROPERTIES
        # PROPERTIES INCLUDE: MARKER ID, MARKER CENTER, BOT or BOBBIN, MARKER ROTATION
        # SUB PROPERTIES INCLUDE: DISTANCE TO CLOSEST BOBBIN, CLOSEST BOBBIN ID, CLOSEST BOBBIN POSITION
        # ============================================
        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame,corners,ids,borderColor=(1,1,1))
            rvecs, tvecs, objPoint = cv2.aruco.estimatePoseSingleMarkers(corners, 0.07, color_cam_matrix, color_dist_coeffs)
            
            for i, idd in enumerate(ids):
                cv2.aruco.drawAxis(frame, color_cam_matrix, color_dist_coeffs, rvecs[i], tvecs[i], 0.02)
                cX = int((corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4)
                cY = int((corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1])/ 4)
                cv2.circle(frame, (cX, cY), 5, (0,255,0), -1)
                cv2.putText(frame, "detected center", (cX - 100, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                



                # ============================================
                # Visualization 
                # ============================================
                # get points for a cube
                if CUBE_VIS:
                    axis = np.float32([[-0.04, -0.04, 0], [-0.04, 0.04, 0], [0.04, 0.04, 0], [0.04, -0.04, 0],
                                [-0.04, -0.04, 0.08], [-0.04, 0.04, 0.08], [0.04, 0.04, 0.08],[0.04, -0.04, 0.08]])
                    imgpts, jac = cv2.projectPoints(axis, rvecs[i], tvecs[i], color_cam_matrix_1280x720, color_dist_coeffs)
                    imgpts = np.int32(imgpts).reshape(-1, 2)
                    # draw cube on frame
                    frame = cv2.drawContours(frame, [imgpts[:4]], -1, (255, 0, 0), 1)
                    for k, j in zip(range(4), range(4, 8)):
                        frame = cv2.line(frame, tuple(imgpts[k]), tuple(imgpts[j]), (255, 0, 0), 1)
                    frame = cv2.drawContours(frame, [imgpts[4:]], -1, (255, 0, 0), 1)
                # ============================================

                # =======================================
                # ADDED FOR DEPTH
                # =======================================
                # Get Distance from centroid Point to Camera
                dist = depth_frame.get_distance(cX, cY)
                adj_dist = dist - .0135 # adjustment for the bobbin height
                #print("Distance is: ", dist)
                #print("adjusted dist is: ", adj_dist)
                # Receive Coordinates
                depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], adj_dist)
                X_coord = "%.1f" % (depth_point_in_meters_camera_coords[0]*1000)
                Y_coord = "%.1f" % (depth_point_in_meters_camera_coords[1]*1000)
                Z_coord = "%.1f" % (depth_point_in_meters_camera_coords[2]*1000)

                x_num = depth_point_in_meters_camera_coords[0]*1000
                y_num = depth_point_in_meters_camera_coords[1]*1000
                z_num = depth_point_in_meters_camera_coords[2]*1000

                cv2.putText(frame, "X: " + X_coord + " Y: " + Y_coord + " Z: " + Z_coord, (cX -130, cY + 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                # =======================================





                # ============================================
                # Data Management 
                # ============================================
                # if the id has not been detected yet, add it to dict and initialize dict values
                # see dict structure above
                strID = f'{idd[0]}'
                if strID not in id_dict.keys():
                    #id_dict[strID] = [idd[0], np.asarray((cX, cY)), [], 0, rvecs[i], tvecs[i], camLocation]
                    id_dict[strID] = [idd[0], np.asarray((cX, cY)), [], 0, rvecs[i], tvecs[i], np.asarray((x_num, y_num, z_num))]
                # if an ID over 20 is detected this is wrong, delete it
                # this is a quick fix for mis detected markers ***FIX
                if id_dict[strID][0] > 15:
                    del id_dict[strID]
    
                # check if the detected id is already in the dict and update other variables too 
                for x in id_dict.keys():
                    if int(strID) == id_dict[x][0]:
                        id_dict[x][1] = np.asarray((cX,cY))                 # update position
                        id_dict[x][2].append(np.asarray((cX,cY)))           # add position to history list
                        id_dict[x][4] = rvecs[i]                            # update rvec
                        id_dict[x][5] = tvecs[i]                            # update tvec
                        id_dict[x][6] = np.asarray((x_num,y_num,z_num))     # update CAMERA position
                    if len(id_dict[x][2]) > 10:                             # if history list is to long (this is for vis. KF only needs most k-1)
                        id_dict[x][2].pop(0)                                # remove the first item from the list

                #print(f'Detected position of marker {id_dict[res[1][i][0]][0]} is {id_dict[res[1][i][0]][1]}')
                # ============================================


        # ============================================
        # Apply KF 
        # ============================================
        if id_dict:
            for x in id_dict.keys():
                if len(id_dict[x][2]) > 1:                                  # if there is a history
                    # apply the Kalman Filter to the history list
                    (filtered_state_means, filtered_state_covariances) = myKF.filter(id_dict[x][2])

                    # assign the estimated position to dict
                    id_dict[x][3] = np.asarray((int(round(filtered_state_means[-1][0])),
                                    int(round(filtered_state_means[-1][2]))))
                    #print(f'Estimated position of marker {id_dict[x][0]} is {id_dict[x][3]}')
                    #print(f'{id_dict[x][3][0]}')
                    #print(f'{id_dict[x][2]}')

                    # draw the last estimated center
                    # this will be the position we use when the marker cannot be detected!!
                    cv2.circle(frame, (id_dict[x][3][0], id_dict[x][3][1]), 5, (0,0,255), -1)
                    cv2.putText(frame, "estimated center", (id_dict[x][3][0] - 100, id_dict[x][3][1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    
                    # draw the predicted history and real history
                    for i, state in enumerate(filtered_state_means):
                        cv2.circle(frame, (int(round(filtered_state_means[i][0])), int(round(filtered_state_means[i][2]))), 2, (0,0,255), -1)
                        #cv2.circle(frame, (id_dict[x][2][i][0], id_dict[x][2][i][1]), 2, (0,255,0), -1)
        # ============================================

            # ============================================
            # Send Data To obj_server
            # ============================================
            obj_dict = deepcopy(id_dict)                                    # make a copy of the id_dict for streaming to obj_server
            for x in obj_dict.keys():
                obj_dict[x][2] = 'null'                                     # get rid of the history list
                for i, item in enumerate(obj_dict[x]):                      # cannot unpack type np array from bytes
                    if type(item) is np.ndarray:                            # check if the item is type np array
                        obj_dict[x][i] = item.tolist()                      # convert to list
            # print('OBJ_DICT_image_client')
            # print(type(obj_dict))
            # print(obj_dict)
            try:
                s.sendall(bytes(f'{obj_dict}', 'utf-8'))                    # send dict to obj_server
                time.sleep(0.12)
            except:
                logging.debug('[image_client.py] Cannot send data...')
            # ============================================


        cv2.imshow("frame", frame)
        # cv2.imshow("marker", gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # ============================================


    cap.release()               # When everything done, release the frame capture
    cv2.destroyAllWindows()     # close open cv visual windows

    logging.debug('Quitting [image_client.py] from keypress [q]')

    s.close()                   # close the socket
    #s.shutdown(socket.SHUT_RDWR)
    
# ============================================

if __name__ == '__main__':
    main()
