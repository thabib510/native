#!/usr/bin/env python
# Listener:
import rospy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from transform_methods import *
from path_init import *
from AStarPathFinder import *

map_array= np.zeros((1,1))
goal = None
map_conversion=None
ready = False

def callback3(data):
    global goal, ready
    #get goal, raise flag and allow current pose to handle
    rospy.loginfo(rospy.get_caller_id()+" goal recieved" )
    goal = data
    ready = True

def callback2(data):
    global goal, ready, map_array, map_conversion
#    rospy.loginfo(rospy.get_caller_id())
    if ready:
        ready = False
        rospy.loginfo("Goal pose frame: "+goal.header.frame_id)
        bottom = (map_conversion[3].x,map_conversion[3].y)
        resolution = map_conversion[2]
        worldSize = (map_conversion[0],map_conversion[1])
        goalPoint = (goal.pose.position.x,goal.pose.position.y)
        startPoint = (data.pose.pose.position.x,data.pose.pose.position.y)
        print "P3DX located at "+str(startPoint)
        #endNode = mapToPGM(bottom,resolution,goalPoint,worldSize)
        endNode = (0,0)
        startNode = mapToPGM(bottom,resolution, startPoint,worldSize)
        newStart = PGMToMap(bottom,resolution,startNode,worldSize)
        print "Re-transformed P3DX "+str(newStart)
        rospy.loginfo("start point: " + str(startNode[0])+","+str(startNode[1]))
        rospy.loginfo("endPoint point: " + str(endNode[0])+","+str(endNode[1]))
        findPath(startNode,endNode,map_array,worldSize,resolution,bottom)
        
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
        row = list(rawArray[start:n])
        for k in range(len(row)):
            if row[k] >25:
                image[i,k]=0
            elif row[k] == -1:
                image[i,k]=-1
            else:
                image[i,k]=256
        start = n+1
    reversed_image = image[::-1]
    map_array = reversed_image #save map
    #pyplot.imshow(reversed_image, pyplot.cm.gray)
    #pyplot.show()  

def listener():
    rospy.init_node('ASTAR',anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.Subscriber("RosAria/pose_map",Odometry, callback2)
    rospy.Subscriber("move_base_simple/goal",PoseStamped,callback3)
    rospy.spin()
#--------------------------------------------------------------
if __name__ == '__main__':
    listener()
