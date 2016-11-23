#!/usr/bin/env python
# Listener:
import rospy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
from transform_methods import *
from path_init import *
from AStarPathFinder import *

map_array= np.zeros((1,1))
goal = None
map_conversion=None
ready = False
mapReady = False


def callback3(data):
    global goal, ready
    #get goal, raise flag and allow current pose to handle
    rospy.loginfo(rospy.get_caller_id()+" goal recieved" )
    goal = data
    if data.orientation.w==1:
        ready = True

def callback2(data):
    global goal, ready, map_array, map_conversion
#    rospy.loginfo(rospy.get_caller_id())
    if ready:
        ready = False
        goalPoint = (goal.position.x,goal.position.y)
        #clear goal node
        map_array[goalPoint[1]-1:goalPoint[1]+1,goalPoint[0]-1:goalPoint[0]+1]=256
        print "dilating.."
        map_array =dilate(map_array)
        print "eroding.."
        map_array=erode(map_array)
        print "more dilating..."
        for i in range(3):
            map_array =dilate(map_array)
        
        #rospy.loginfo("Goal pose frame: "+goal.header.frame_id)
        bottom = (map_conversion[3].x,map_conversion[3].y)
        resolution = map_conversion[2]
        worldSize = (map_conversion[0],map_conversion[1])
        
        
        startPoint = (data.pose.pose.position.x,data.pose.pose.position.y)
        endNode = mapToPGM(bottom,resolution,goalPoint,worldSize)
        startNode = mapToPGM(bottom,resolution, startPoint,worldSize)
        rospy.loginfo("start point: " + str(startNode[0])+","+str(startNode[1]))
        rospy.loginfo("endPoint point: " + str(endNode[0])+","+str(endNode[1]))
        findPath(startNode,endNode,map_array,worldSize,resolution,bottom)
        
        #endNode = Node(mapToPGM(bottom,resolution,goalPoint,worldSize),0,0)
        #startNode = Node(mapToPGM(bottom,resolution, startPoint,worldSize),0,0)
        #paintNode(Node(startNode,0,0),2,map_array)
        #paintNode(Node(endNode,0,0),2,map_array)
        #pyplot.imshow(map_array, pyplot.cm.gray)
        #pyplot.show()  
        
    return 

def callback(data):
    global map_array, map_conversion
    rospy.loginfo(rospy.get_caller_id()+" map data received") #acknowledge for degub purposes
    width=data.info.width
    height=data.info.height
    image=np.zeros((height,width))
    map_conversion=(width,height,data.info.resolution,data.info.origin.position)
    rawArray = data.data
    length = len(rawArray)
    start = 0
    for i in range(height):
        n=(i+1)*width
       #image[i] = rawArray[start:n]
        row = list(rawArray[start:n])
        for k in range(len(row)):
            if row[k] >25:
                image[i,k]=5
            elif row[k]==-1:
                image[i,k]=-1
            else:
                image[i,k]=256
        start = n+1
    #map_array = image
    #mapReady=True
    #print "storing map.."
    reversed_image = image[::-1]
    map_array = reversed_image #save map
    #map_array = data.mapArray #save map


#--------------------------------------------------------------------------------------
# Best dilate by one solution
def dilate(image):
    height = len(image)
    width = len(image[0])
    for i in range(height):
        
        for j in range(width):
            if (image[i][j] == 5):
                if (i>0 and image[i-1][j]==256):
                    image[i-1][j] = 8
                if (j>0 and image[i][j-1]==256):
                    image[i][j-1] = 8
                if (i+1<height and image[i+1][j]==256):
                    image[i+1][j] = 8
                if (j+1<width and image[i][j+1]==256):
                    image[i][j+1] = 8
    image[image==8]=5
    return image

def erode(image):
    height = len(image)
    width = len(image[0])
    for i in range(height):
        
        for j in range(width):
            if (image[i][j] == 256):
                if (i>0 and image[i-1][j]<6):
                    image[i-1][j] = 2
                if (j>0 and image[i][j-1]<6):
                    image[i][j-1] = 2
                if (i+1<height and image[i+1][j]<6):
                    image[i+1][j] = 2
                if (j+1<width and image[i][j+1]<6):
                    image[i][j+1] = 2
    image[image==2]=256
    return image


# n^2 solution with Manhattan oracle
def dilateByK(image, k):
    image = manhattan(image)
    image[image <= k] = 5
    image[image >k ] = 256
    return image

# O(n^2) solution to find the Manhattan distance to "on" pixels in a two dimension array
def manhattan(image):
    # traverse from top left to bottom right
    for i in range(len(image)):
        for j in range(len(image[i])):
            if (image[i][j] == 5):
                # first pass and pixel was on, it gets a zero
                image[i][j] = 0
            else:
                #// pixel was off
                #// It is at most the sum of the lengths of the array
                #// away from a pixel that is on
                image[i][j] = len(image) + len(image[i])
                #// or one more than the pixel to the north
                if (i>0):
                    image[i][j] = min(image[i][j], image[i-1][j]+1)
                #// or one more than the pixel to the west
                if (j>0):
                    image[i][j] = min(image[i][j], image[i][j-1]+1)
    #// traverse from bottom right to top left
    for i in range(len(image)-1,-1,-1):
        for j in range(len(image[i])-1,-1,-1):
            #// either what we had on the first pass
            #// or one more than the pixel to the south
            if (i+1<len(image)):
               image[i][j] = min(image[i][j], image[i+1][j]+1)
            #// or one more than the pixel to the east
            if (j+1<len(image[i])):
               image[i][j] = min(image[i][j], image[i][j+1]+1)
    return image
#--------------------------------------------------------------------------------------

def listener():
    rospy.init_node('ASTAR',anonymous=False)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.Subscriber("RosAria/pose_map",Odometry, callback2)
    rospy.Subscriber("/Target",Pose,callback3)
    rospy.spin()
#--------------------------------------------------------------
if __name__ == '__main__':
    listener()
