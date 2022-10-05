
"""library for bot and bobbin classes

This script contains the bot and bobbin class as we as some useful functions for 
rotation calculations and conversions.

"""

import numpy as np
from math import hypot, atan2, degrees, sqrt, pi
import cv2


# ============================================
# OTHER FUNCTIONS
# ============================================
def listOfIntToListOfStrings(inList):
    '''
    take a list of ints and convert to a list of str
    '''
    for i, item in enumerate(inList):
        inList[i] = str(item)
    return inList
# ============================================


# ============================================
# ROTATION AND CONVERSION FUNCTIONS
# ============================================
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    """
    check if a matrix is a valid rotation matrix
    """
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# https://learnopencv.com/rotation-matrix-to-euler-angles/
# The result is the same as MATLAB except the order of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
    """
    convert rotation matrix to euler angles
    """

    assert(isRotationMatrix(R))
    
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else :
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# convert radians to degrees
def radiansToDegrees(array, bool):
    """
    convert radiants to degrees. if True round the result
    """
    out = [0,0,0]
    if bool:
        for i, num in enumerate(array):
            out[i] = round((num * 180/pi), 0)
    else:
        for i, num in enumerate(array):
            out[i] = num * 180/pi

    return out

# convert negative degrees to positive degrees
def positiveDegrees(degs):
    """ 
    takes a single np.float64 and converts from (-180-180) to (0-360)
    """
    int(degs)
    if degs < 0:
        degs += 360
    elif degs > 360:
        degs -= 360

    return degs
# ============================================

# ============================================
# CONTROLLER FUNCTIONS i.e. originally computed in the bot object, now at the controller
# ============================================
def findAngleToWaypoint(bot_location, waypoint):
    '''
    find angle to waypoint (x,y) from current bot location (x,y)
    '''
    dx = bot_location[0] - waypoint[0]
    dy = bot_location[1] - waypoint[1]
    rads = atan2(dy, dx)
    degs = degrees(rads)
    if degs < 0:
        degs += 360
    elif degs > 360:
        degs -= 360
    degs = round(degs, 0)
    target_angle = degs
    return target_angle


def computeRequiredRotation(bot_rotation, target_angle):
    '''
    based on the bots current rotation and the target angle
    compute the required rotation angle to align with target
    '''
    required_rotation = bot_rotation - target_angle
    return required_rotation


def findDistanceToWaypoint(bot_location, waypoint):
    '''
    function to compute distance between two (x,y) coordinates
    P(x1, y1) Q(x2, y2)
    P = self.location, Q = waypoint
    '''
    distance = sqrt((waypoint[0] - bot_location[0])**2 + (waypoint[1] - bot_location[1])**2)
    waypoint_distance = distance
    return waypoint_distance

# ============================================

# ============================================
# DEFINE CLASSES
# ============================================
class bot():
    '''
    a class to store information pertaining to the bots
    '''
    def __init__(self, idNum):
        self.id = idNum                     # marker id number
        self.type = 'bot'                   # bot or bobbin
        self.rotation = 0                   # current rotation wrt camera
        self.location = (0,0)               # current position (pixels)
        self.estLocation = (0,0)             # estimated position from KF
        self.camLocation = (0,0,0)
        self.rvec = np.zeros((1,3))         # current rvec
        self.tvec = np.zeros((1,3))         # current tvec

        self.distances = []                 # list of list of tuples (bobbinID, bobbinLocation, distance)
        self.closestBobbin = (0,[0,0],0)    # the closest bobbin as tuple (id, position, distance)
        self.targetAngle = 0                # the angle from bot to closest bobbin
        self.requiredRotation = 0           # rotation required to align bot with bobbin

        self.waypoints = []                 # list to store waypoints as (x,y) pixel tuples
        #self.waypointCounter = 0            # counter to store how many waypoints have been visited
        self.waypointDistance = 0           # var to store distance to current waypoint

        

        self.tasks = {}                     # dict to store "tasks"
        #self.detectflag = True

    def computeBotRotation(self):
        '''
        compute rotation of marker wrt camera
        and assign it to object
        '''
        rotMat = np.zeros(shape=(3,3))
        cv2.Rodrigues(self.rvec, rotMat)
        markerEulerRot_rads = rotationMatrixToEulerAngles(rotMat)  #convert rotation matrix to euler angles (x,y,z)
        markerEulerRot_degs = radiansToDegrees(markerEulerRot_rads, True) # convert euler angles from rad to degs      
        self.rotation = positiveDegrees(markerEulerRot_degs[2])


    def findDistanceToAllBobbins(self, bobbinObjectDict):
        '''
        find distances to all bobbins
        store as a list of tuples (bobbinID, bobbinLocation, distance)
        '''
        distances = [] # list of tuples (bobbinID, bobbinLocation, distance)
        for x in bobbinObjectDict:
            thisBobbin = bobbinObjectDict[x]
            dist = hypot((self.location[0] - thisBobbin.location[0]), (self.location[1] - thisBobbin.location[1]))
            distances.append((thisBobbin.id[0], thisBobbin.location, dist))
        self.distances = distances


    def findClosestBobbin(self):
        '''
        find the closest bobbin to the bot
        '''
        # sort the list by distance and retrieve closest bobbin (bobbinID, bobbinLocation, dist)
        sortedDistances = sorted(self.distances, key=lambda tup: (tup[2]))
        if len(sortedDistances) > 0:
            self.closestBobbin = sortedDistances[0]

    def findAngleToClosestBobbin(self):
        '''
        find the angle to the closest bobbin
        aka target angle
        '''
        target = self.closestBobbin[1]
        dx = self.location[0] - target[0]
        dy = self.location[1] - target[1]
        rads = atan2(dy, dx) 
        degs = degrees(rads)
        if degs < 0:
            degs += 360
        elif degs > 360:
            degs -= 360
        degs = round(degs, 0)
        self.targetAngle = degs

    def computeRequiredRotation(self):
        '''
        based on the bots current rotation and the target angle
        compute the required rotation angle to align with target
        '''
        self.requiredRotation = self.rotation - self.targetAngle
        #return self.rotation - self.targetAngle

    def findAngleToWaypoint(self, waypoint):
        '''
        find angle to waypoint (x,y)
        '''
        target = waypoint
        dx = self.location[0] - waypoint[0]
        dy = self.location[1] - waypoint[1]
        rads = atan2(dy, dx)
        degs = degrees(rads)
        if degs < 0:
            degs += 360
        elif degs > 360:
            degs -= 360
        degs = round(degs, 0)
        self.targetAngle = degs
        #return degs

    def findDistanceToWaypoint(self, waypoint):
        '''
        function to compute distance between two (x,y) coordinates
        P(x1, y1) Q(x2, y2)
        P = self.location, Q = waypoint
        '''
        distance = sqrt((waypoint[0] - self.location[0])**2 + (waypoint[1] - self.location[1])**2)
        self.waypointDistance = distance




        
class bobbin(bot):  
    '''
    a class to store information about bobbins
    '''
    def __init__(self, idNum):
        self.id = idNum
        self.type = 'bobbin'
        self.rotation = 0
        self.location = (0,0)
        self.estLocation = (0,0)
        self.camLocation = (0,0,0)
        self.rvec = np.zeros((1,3))
        self.tvec = np.zeros((1,3))

# ============================================
# ============================================
# END CLASS SECTION
# ============================================