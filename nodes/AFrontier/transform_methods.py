#!/usr/bin/env python
#tf
from numpy.linalg import inv
from AStarPathFinder import *
from geometry_msgs.msg import Pose

def mapToPGM(bottom,resolution,point,worldSize):
    newX = int((abs(bottom[0])+point[0])/resolution)
    newY = worldSize[1]-int(abs(bottom[1])/resolution)-int(point[1]/resolution)
    return (newX,newY)
def PGMToMap(bottom,resolution,point,worldSize):
    mapX = point[0]*resolution - bottom[0]
    mapY = worldSize[1]*resolution -point[1]*resolution - bottom[1]
    return (mapX,mapY)
def getListOfGoals(path,origin,worldSize,resolution):
    listOfGoals=[]
    for node in path.path:
        location=(node.getX(),node.getY(),0,1)
        pose = PGMToMap(origin,resolution,location,worldSize)
        msg = Pose()
        msg.position.x=pose[0]
        msg.position.y=pose[1]
        msg.position.z=0
        listOfGoals.append(msg)
    return listOfGoals
