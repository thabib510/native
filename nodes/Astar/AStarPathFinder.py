#!/usr/bin/env python
import roslib
import re
import numpy
from matplotlib import pyplot

#-------------------------------------------------------------------------
# for debuggin only!
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
#-------------------------------------------------------------------------------------
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
    dstY = abs(nodeB.gridY-nodeA.gridY)
    if(dstX > dstY):
        return (14*dstY + 10*(dstX-dstY))
    return (14*dstX + 10*(dstY-dstX))

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
                #nodeNeighbor.setgCost(getDistance(nodeNeighbor,node)) # cost to this guy.
                nodeNeighbor.sethCost(getDistance(nodeNeighbor,targetNode))
                neighbors.append(nodeNeighbor)
    #check second neighbors
    for node in neighbors:
        for y in range(-1,2):
            for x in range (-1,2):
                if (x==0 and y==0):
                    continue
                checkY = node.gridY +y
                checkX = node.gridX +x
                if (checkX >=0 and checkX<grid.gridSizeX and checkY>= 0 and checkY<grid.gridSizeY):
                    if (not grid.grid[checkY][checkX].walkable): # if any of my second neighbors is a "wall" then make me more costly
                        node.addTax(7)
    if(not neighbors[1].walkable):  
        neighbors[0].addTax(3)
        neighbors[2].addTax(3)
    if(not neighbors[3].walkable):
        neighbors[0].addTax(3)
        neighbors[5].addTax(3)
    if(not neighbors[4].walkable):
        neighbors[2].addTax(3)
        neighbors[7].addTax(3)
    if(not neighbors[6].walkable):
        neighbors[5].addTax(3)
        neighbors[7].addTax(3)
    return neighbors

def nodeFromWorldPoint(position,grid):
    percentX = float(position[0])/grid.worldSize[0] 
    percentY = float((grid.worldSize[1]-position[1]))/grid.worldSize[1] 
    x = int((grid.gridSizeX-1)*percentX) 
    y = int((grid.gridSizeY-1)*percentY) 
    print "positions: "+str(position)+"; on grid: ",x,y
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
        self.tax=0
        self.gcost=0 # 0 means this node has not been evaluated
        
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
    def addTax(self,addedTax):
        self.tax=self.tax+addedTax
    def getfCost(self):
        return self.gcost+self.hcost+self.tax
    def isDiagonal(self):
        return self.diagonal
    def setDiagonal(self,diagonality):
        self.diagonal = diagonality
    def setParentNode(self,node):
        self.parent = node
    def isWalkable(self, plane):
        for i in range (len(plane)):
            for k in range (len(plane[i])):
                if plane[i][k] <230:
                    return False
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
                frontierPlane = numpy.zeros((nodeDiameter,nodeDiameter))
                unoccupiedPlane = numpy.zeros((nodeDiameter,nodeDiameter))
                occupiedPlane = numpy.zeros((nodeDiameter,nodeDiameter))

                frontierPlane[0:nodeDiameter,0:nodeDiameter]=-1
                occupiedPlane[0:nodeDiameter,0:nodeDiameter]=5
                unoccupiedPlane[0:nodeDiameter,0:nodeDiameter]=230
                frontierTestArray = nodePlane == frontierPlane
                unoccupiedTestArray = nodePlane > unoccupiedPlane
                occupiedTestArray = nodePlane == occupiedPlane
                newNode=Node((xpoint,ypoint),i,j)
                if unoccupiedTestArray.all():
                #all pixels are white:
                    newNode.walkable=True
                elif (occupiedTestArray.any()):
                # if any are occupied:
                    newNode.walkable=False
                elif(frontierTestArray.all()):
                #if all equal the unexplored plain, not frontier:
                    newNode.walkable=False
                else:
                # else, there is no occupied, only white and gray:
                    newNode.walkable=True
                #newNode.setWalkable(nodePlane)
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
        startNode.sethCost(getDistance(startNode,targetNode))
        #startNode.setgCost(0)
        self.startNode=startNode
        self.targetNode=targetNode
        openSet=[startNode]
        closedSet=[]
        while(len(openSet)>0):
            currentNode = openSet[0]
            for i in range(len(openSet)):
                if(openSet[i].getfCost()<currentNode.getfCost() or (openSet[i].getfCost() == currentNode.getfCost() and openSet[i].hcost < currentNode.hcost)): 
                    currentNode = openSet[i]
            openSet.remove(currentNode)
            closedSet.append(currentNode)
            #self.mappy = paintNode(currentNode,self.mappy)
            if (currentNode.worldPosition == targetNode.worldPosition):
                self.path = retracePath(startNode,targetNode)
                self.pathExist=True
                return
            for nodeNeighbor in getNeighbors(currentNode,grid,startNode,targetNode):
                if ((not nodeNeighbor.walkable) or (nodeNeighbor in closedSet)):
                    continue                        
                newMovementCostToNeighbor = currentNode.gcost + getDistance(currentNode,nodeNeighbor) #
                if(nodeNeighbor.gcost == 0 or newMovementCostToNeighbor< nodeNeighbor.gcost): 
                    nodeNeighbor.setgCost(newMovementCostToNeighbor)
                    #nodeNeighbor.sethCost(getDistance(nodeNeighbor, targetNode))
                    nodeNeighbor.setParentNode(currentNode)
                #openSet.append(nodeNeighbor)
                if(not (nodeNeighbor in openSet)): 
                    openSet.append(nodeNeighbor)
 
