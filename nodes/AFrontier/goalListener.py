#!/usr/bin/env python
# Listener:
import rospy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Header
import numpy as np
from transform_methods import *
from path_init import *
from FrontierFinder import *

map_array= np.zeros((1,1))
goal = None
map_conversion=None
grid = None
ready = False
mapReady = False
mapMetaData = MapMetaData()
header = Header()


def callback3(data):
    global goal, ready
    #get goal, raise flag and allow current pose to handle
    rospy.loginfo(rospy.get_caller_id()+" goal recieved" )
    if data.data =="Ready":
        ready = True
        print "task accepted..."

def callback2(data):
    global goal, ready, map_array, map_conversion
    #rospy.loginfo(rospy.get_caller_id())
    #if ready and mapReady:
    if ready:
        ready = False
        mapReady = False
        print "dilating.."
        map_array =dilate(map_array)
        print "eroding.."
        map_array=erode(map_array)
        print "more dilating.."
        for i in range(3):
            map_array =dilate(map_array)
        #map_array[map_array==-1]=180
        #pyplot.imshow(map_array, pyplot.cm.gray)
        #pyplot.show()  
        #map_array[map_array==180]=-1
        #publishMapArray(map_array)
        #rospy.loginfo("Goal pose frame: "+goal.header.frame_id)
        bottom = (map_conversion[3].x,map_conversion[3].y)
        resolution = map_conversion[2]
        worldSize = (map_conversion[0],map_conversion[1])
        #goalPoint = (goal.pose.position.x,goal.pose.position.y)
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
    global map_array, map_conversion, grid, mapReady, mapMetaData, header
    now = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id()+" map data received")
    print data.header.stamp.secs, " and ", now.secs
    if(True):
    #if ((now.secs - data.header.stamp.secs) < 30):
        #rospy.loginfo(rospy.get_caller_id()+" map data received") #acknowledge for degub purposes
        print "inside"
        header = data.header
        mapMetaData = data.info
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
                    image[i,k]=5
                elif row[k] == -1:
                    image[i,k]=-1
                else:
                    image[i,k]=256
            start = n+1
        reversed_image = image[::-1]
        map_array = reversed_image 
#-----------------------------------------------------------------------------------------
def publishMapArray(msg):
    global mapMetaData, header
    msg[msg==5]=50
    msg[msg==256]=0
    processedMap = OccupancyGrid()
    data =[]
    for i in range(len(msg)):
        #processedMap.data.append(msg[i])    
        for j in range(len(msg[i])):
            data.append(int(msg[i][j]))
    processedMap.info = mapMetaData
    processedMap.header = header
    processedMap.data=data
    #processedMap = OccupancyGrid(header,mapMetaData,data)
    pub = rospy.Publisher('ProcessedMap', OccupancyGrid, queue_size = 10)
    emptyMsg = OccupancyGrid()
    pub.publish(processedMap)
    print "publishing..."
    pub.publish(processedMap)
    msg[msg==50]=5
    msg[msg==0]=256
  
#-----------------------------------------------------------------------------------------
#eroding and dialating methods:
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
#------------------------------------------------------------------------------ 


def listener():
    rospy.init_node('Frontier',anonymous=False)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.Subscriber("RosAria/pose_map",Odometry, callback2)
    rospy.Subscriber("GoalRequest",String,callback3)
    rospy.spin()
#--------------------------------------------------------------
if __name__ == '__main__':
    listener()
