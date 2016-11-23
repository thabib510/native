#!/usr/bin/env python
import roslib
import re
import numpy
import rospy
from numpy.linalg import inv
from matplotlib import pyplot
from geometry_msgs.msg import Pose
from FrontierFinder import *
#from pgm_reader import *
from transform_methods import *
#from native.msg import path as pathMsg
import datetime
#---------------------------------------------------------------------------------------------------
#empty message:
EmptyMessage = Pose()
# drawing methods:
def drawPath(path,image,radius,gridWorldSize):
    for node in path.path:
        nodeDiameter=radius * 2
        nodeRadius = radius
        xpoint = node.gridX*nodeDiameter+nodeRadius              
        ypoint = gridWorldSize[1] - (node.gridY*nodeDiameter+nodeRadius) 
        x1 = xpoint-nodeRadius
        x2 = xpoint+nodeRadius
        y1 = ypoint-nodeRadius
        y2 = ypoint+nodeRadius
        image[y1:y2,x1:x2] =100
    return image

def paintNode(node,radius, image):
    nodeRadius = radius
    xpoint = node.getX()
    ypoint = node.getY()
    x1 = xpoint-nodeRadius
    x2 = xpoint+nodeRadius
    y1 = ypoint-nodeRadius
    y2 = ypoint+nodeRadius
    image[y1:y2,x1:x2] =10
    return image
#---------------------------------------------------------------------------------------------------
def PathPublisher(msg):
    global EmptyMessage
    pub = rospy.Publisher('Target', Pose, queue_size = 10)
    pub.publish(EmptyMessage)
    rate = rospy.Rate(2)
    rate.sleep()
    pub.publish(msg)
        
#---------------------------------------------------------------------------------------------------
def findPath(start,end,imageMap, worldSize, resolution, bottom):
    image=imageMap
    #nodeRadius = 2
    nodeRadius = int(0.4/resolution)
    if nodeRadius<1:
        nodeRadius = 1
    # create a grid and compute path:
    print datetime.datetime.now().time()
    print "creating Grid... "
    grid = Grid(nodeRadius,image)
    print "finding next frontier..."
    print datetime.datetime.now().time()
    path = Pathfinding(start,end,grid,image)
    print datetime.datetime.now().time()
    print "frontier found: ",path.pathExist
    # publish path:
    if(path.pathExist):
        #listOfGoals=getListOfGoals(path,bottom,worldSize,resolution)
        pubMessage = Pose()
        point = (path.targetNode.getX(),path.targetNode.getY())
        location = PGMToMap(bottom,resolution,point,worldSize)
        pubMessage.position.x = location[0]
        pubMessage.position.y = location[1]
        pubMessage.orientation.w=1.0
        PathPublisher(pubMessage)
#---------------------------------------------------------------------
# read map from file:
#    image = read_pgm("3rd-Floor-2D-Map-Test-2.pgm", byteorder='<')
#    image.flags.writeable=True
#--------------------------------------------------------------------  
     
   

