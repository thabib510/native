#!/usr/bin/env python
import roslib
import re
import numpy
from matplotlib import pyplot

#-------------------------------------------------------------------------
# methods associated with A*
def retracePath(startNode, endNode):
    currentNode = endNode
    path =[]
    while(currentNode != startNode):
        path.append(currentNode)
        currentNode = currentNode.parent
    path.reverse()
    return path

def getDistance(nodeA, nodeB):
    dstX = abs(nodeB.gridX-nodeA.gridX)
    dstY = abs(nodeB.gridY-nodeB.gridY)
    if(dstX > dstY):
        return (10*dstY + 10*(dstX-dstY))
    return (10*dstX + 10*(dstY-dstX))

def getNeighbors(node,grid,startNode,targetNode):
    neighbors = []
    for y in range(-1,2):
        for x in range(-1,2):
            if (x==0 and y == 0):
                continue
            checkY = node.gridY + y
            checkX = node.gridX + x
            if (checkX >=0 and checkX<grid.gridSizeX and checkY>= 0 and checkY<grid.gridSizeY):
                nodeNeighbor = grid.grid[checkY][checkX]
                nodeNeighbor.setgCost(getDistance(nodeNeighbor,startNode))
                # in the frontier case there is no Hcost! we are searching unboundely..
                #nodeNeighbor.sethCost(getDistance(nodeNeighbor,targetNode))
                #nodeNeighbor.sethCost(0)
                neighbors.append(nodeNeighbor)
    return neighbors

def nodeFromWorldPoint(position,grid):
    percentX = float(position[0])/grid.worldSize[0] 
    percentY = float((grid.worldSize[1]-position[1]))/grid.worldSize[1] 
    x = int((grid.gridSizeX-1)*percentX) 
    y = int((grid.gridSizeY-1)*percentY) 
    #print "positions: "+str(position)+"; on grid: ",x,y
    return grid.grid[y][x] 
    
#---------------------------------------------------------------------------
# Classes
# node Class:
# worldPosition is a 2d vector (tuple): (x,y)
class Node(object):
    def __init__(self,worldPosition,x,y):
        self.worldPosition = worldPosition
        self.gridX = x
        self.gridY = y
        self.isFrontier = False        
    def getX(self):
        return self.worldPosition[0] 
    def getY(self):
        return self.worldPosition[1] 
    def setWalkable(self, fullNode):
        self.walkable = self.isWalkable(fullNode)
    def setgCost(self,cost):
        self.gcost=cost
    def sethCost(self,cost):
        self.hcost=cost
    def getfCost(self):
        return self.gcost+self.hcost
    def setParentNode(self,node):
        self.parent = node
    def isWalkable(self, plane):
        f=0
        for i in range (len(plane)):
            for k in range (len(plane[i])):
                #check for the case we don't know: 
                if plane[i][k] <230 and plane[i][k]>-1:
                    self.isFrontier=False
                    return False
                if plane[i][k] == -1:
                    f=f+1
                    if f>14:
                        #print "Frontier candidate found!",f     
                        self.isFrontier=True
                     
        return True
    
    
# Grid Class
# gridWorldSize is a 2d vector (tuple): (x,y)       
class Grid(object):
    def __init__(self,nodeRadius,image):
        self.nodeRadius = nodeRadius
        gridWorldSize = (len(image[0]),len(image))
        nodeDiameter = nodeRadius*2
        self.gridSizeX=(gridWorldSize[0]/nodeDiameter)
        self.gridSizeY=(gridWorldSize[1]/nodeDiameter)
        grid = [[0 for x in range(self.gridSizeX)]for y in range(self.gridSizeY)]
        for j in range (self.gridSizeY):
            for i in range(self.gridSizeX):
                xpoint = i*nodeDiameter+nodeRadius
                ypoint = gridWorldSize[1] - (j*nodeDiameter+nodeRadius)
                x1 = xpoint-nodeRadius
                x2 = xpoint+nodeRadius
                y1 = ypoint-nodeRadius
                y2 = ypoint+nodeRadius
                nodePlane = image[y1:y2,x1:x2]
                newNode=Node((xpoint,ypoint),i,j)
                newNode.setWalkable(nodePlane)
                grid[j][i]= newNode
        self.grid=grid
        self.worldSize = gridWorldSize


# path finding method/Class            
class Pathfinding(object):
    def __init__(self,startPosition, targetPosition,grid,image):
        self.mappy=image
        self.pathExist=False
        startNode = nodeFromWorldPoint(startPosition,grid)
        targetNode = nodeFromWorldPoint(targetPosition,grid)
        #startNode.sethCost(getDistance(startNode,targetNode))
        startNode.sethCost(150)#albitrarily large number
        startNode.setgCost(0)
        self.startNode=startNode
        #self.targetNode=targetNode
        openSet=[startNode]
        closedSet=[]
        while(len(openSet)>0):
            currentNode = openSet[0]
            for i in range(len(openSet)):
                if(openSet[i].getfCost()<currentNode.getfCost() or (openSet[i].getfCost() == currentNode.getfCost() and openSet[i].hcost < currentNode.hcost)):
                    currentNode = openSet[i]
            openSet.remove(currentNode)
            closedSet.append(currentNode)
            #if (currentNode.worldPosition == targetNode.worldPosition):
            if (currentNode.isFrontier):
                self.path = retracePath(startNode,currentNode)
                self.pathExist=True
                self.targetNode=currentNode
                print "final frontier: ",currentNode.getX(),", ",currentNode.getY()
                return
            for nodeNeighbor in getNeighbors(currentNode,grid,startNode,targetNode):
                if ((not nodeNeighbor.walkable) or (nodeNeighbor in closedSet)):
                    continue                
                newMovementCostToNeighbor = currentNode.gcost + getDistance(currentNode,nodeNeighbor)
                nodeNeighbor.setgCost(newMovementCostToNeighbor)
                #nodeNeighbor.sethCost(getDistance(nodeNeighbor, targetNode))
                nodeNeighbor.sethCost(0)
                nodeNeighbor.setParentNode(currentNode)
                if(not nodeNeighbor in openSet):
                    openSet.append(nodeNeighbor)
 
