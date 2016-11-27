#!/usr/bin/env python
# Clean map:
import rospy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Bool
import numpy as np
from transform_methods import *
from path_init import *
from FrontierFinder import *

map_array= np.zeros((1,1))
goal = None
map_conversion=None
grid = None
ready = False
oldMap = False
processing = False
mapMetaData = MapMetaData()
header = Header()
obstacleList=[]
raw_map=OccupancyGrid()

def callback3(data):
    global obstacleList
    if (not data.position in obstacleList):
        obstacleList.append(data)
        
def callback2(data):
    global map_array, grid, mapMetaData, header, oldMap, ready, raw_map, processing
    if data.data:
        processing = True
        oldMap = False
        header = raw_map.header
        mapMetaData = raw_map.info
        width=raw_map.info.width
        height=raw_map.info.height
        image=np.zeros((height,width))
        rawArray = raw_map.data
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
        print "dilating.."
        map_array =dilate(map_array)
        print "eroding.."
        map_array=erode(map_array)
        print "more dilating.."
        for i in range(3):
        map_array =dilate(map_array)
        map_array[map_array==-1]=180
        pyplot.imshow(map_array, pyplot.cm.gray)
        pyplot.show()  
        map_array[map_array==180]=-1
        oldMap = True
        publishMapArray(map_array)
        processing = False

def callback(data):
    global raw_map
    if not processing:
        raw_map=data

#----------------------------------------------------------------------------------------------------
	
def publishMapArray(msg):
    global mapMetaData, header, oldMap
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
    pub = rospy.Publisher('ProcessedMap', OccupancyGrid, queue_size = 10)
    rate = rospy.Rate(10) # 10hz
    while (not rospy.is_shutdown() and oldMap):
        str_log = "message sent at: %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(processedMap)
        rate.sleep()
    print "publishing..."
    pub.publish(processedMap)
    msg[msg==50]=5
    msg[msg==0]=256
#-----------------------------------------------------------------------------------------------------
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
    rospy.init_node('MapCleaner',anonymous=False)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.Subscriber("Done",Bool, callback2)
    rospy.Subscriber("add_obstacle",Pose,callback3)
    rospy.spin()
#--------------------------------------------------------------
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
	    pass
