#!/usr/bin/env python
import roslib
import re
import numpy
import rospy
from numpy.linalg import inv
from matplotlib import pyplot
from geometry_msgs.msg import Pose
from AStarPathFinder import *
#from pgm_reader import *
from transform_methods import *
from native.msg import path as pathMsg
#---------------------------------------------------------------------------------------------------
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
    pub = rospy.Publisher('Path', pathMsg, queue_size = 10)
    pub.publish(msg)
    #rate = rospy.Rate(2)
    #while not rospy.is_shutdown():
        #pub.publish(msg)
        #rate.sleep()
#---------------------------------------------------------------------------------------------------
def findPath(start,end,imageMap, worldSize, resolution, bottom):
    image=imageMap
    #nodeRadius = 2
    nodeRadius = int(0.3/resolution)
    if nodeRadius<1:
        nodeRadius = 1
    # create a grid and compute path:
    grid = Grid(nodeRadius,image)
    path = Pathfinding(start,end,grid,image)
    # draw the path for demonstration purposes only:
    if(path.pathExist):
        image = drawPath(path,image,nodeRadius, grid.worldSize)
        image = paintNode(path.startNode,nodeRadius,image)
        image = paintNode(path.targetNode,nodeRadius,image)
    pyplot.imshow(image, pyplot.cm.gray)
    pyplot.show()
    print "path was found: ",path.pathExist
    # publish path:
    if(path.pathExist):
        listOfGoals=getListOfGoals(path,bottom,worldSize,resolution)
        pubMessage = pathMsg()
        pubMessage.poses = listOfGoals
        PathPublisher(pubMessage)
#---------------------------------------------------------------------
# read map from file:
#    image = read_pgm("3rd-Floor-2D-Map-Test-2.pgm", byteorder='<')
#    image.flags.writeable=True
#--------------------------------------------------------------------  
     
   

