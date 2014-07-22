import numpy as np
from Hex import *
from HexaUtils import calcHexaWidthHeight, isHexInList
from TopologyGraph import *

FlatDirectionList = ["U", "D", "NE", "NW", "SE", "SW"]
PointyDirectionList = ["E", "W", "NE", "NW", "SE", "SW"]


class HexagonalMap(object):
    
    def __init__(self, x_num, y_num, side, orientation):
        self.x_num = x_num
        self.y_num = y_num
        self.side = side
        self.orientation = orientation
        self.hexes = []
        self.initMap()
        
    def initMap(self):
        h, r = calcHexaWidthHeight(self.side)
        if self.orientation == "FLAT":
            self.cellWidth = self.side + h + h
            self.cellHeight = r + r
            
            self.width = (self.x_num*(self.cellWidth+self.side)) + h
            if self.y_num%2 == 1:
                self.height = int((self.y_num-1)/2) * self.cellHeight + self.cellHeight
            else:
                self.height = (int(self.y_num/2) * self.cellHeight) + r
        elif self.orientation == "POINTY":
            self.cellWidth = r + r
            self.cellHeight = self.side + h + h
            
            self.width = (self.x_num * self.cellWidth) + r
            if self.y_num%2 == 1:
                self.height = int((self.y_num-1)/2) * (self.cellHeight+self.side+h) + h
            else:
                self.height = int(self.y_num/2) * (self.cellHeight+self.side) + h        
            
        for i in range(self.x_num):
            hex_array = []
            for j in range(self.y_num):
                
                if self.orientation=="FLAT":
                    if j%2 == 0:
                        x_offset = h +  i*(self.side+self.side+h+h)
                        y_offset = self.cellHeight * j / 2
                    else:
                        x_offset = self.side+h+h + i*(self.side+self.side+h+h)
                        y_offset = r + self.cellHeight * (j-1) / 2
                elif self.orientation=="POINTY":
                    if j%2 == 0:
                        x_offset = i*self.cellWidth + r
                        y_offset = (self.side+self.cellHeight) * int(j/2)
                    else:
                        x_offset = (i+1)*self.cellWidth
                        y_offset = int((j-1)/2)*(self.cellHeight+self.side) + self.side+h
                
                hx = Hex(x_offset, y_offset, self.side, self.orientation, h, r)
                hex_array.append(hx)
                
            self.hexes.append(hex_array) 
            
    def getHex(self, x_idx, y_idx):
        if x_idx < 0 or x_idx > self.x_num-1:
            return None
        if y_idx < 0 or y_idx > self.y_num-1:
            return None        
        return self.hexes[x_idx][y_idx]
    
    
    def findHex(self, pos_x, pos_y):
        for i in range(self.x_num):
            for j in range(self.y_num):
                if True == self.hexes[i][j].insideHex(pos_x, pos_y):
                    return [i, j]
        return None
    
    def isConnected(self, hexIdxA, hexIdxB):      
        
        if self.orientation=="POINTY":
            if hexIdxA[1]==hexIdxB[1]:
                if hexIdxA[0]==hexIdxB[0]-1 or hexIdxA[0]==hexIdxB[0]+1 or hexIdxA[0]==hexIdxB[0]:
                    return True
            elif hexIdxA[1]==hexIdxB[1]-1 or hexIdxA[1]==hexIdxB[1]+1:
                if hexIdxB[1]%2 == 0:
                    if hexIdxA[0]==hexIdxB[0] or hexIdxA[0]==hexIdxB[0]-1:
                        return True
                elif hexIdxB[1]%2 == 1:
                    if hexIdxA[0]==hexIdxB[0] or hexIdxA[0]==hexIdxB[0]+1:
                        return True    
            
        elif self.orientation=="FLAT":
            
            if hexIdxA[1]==hexIdxB[1] or hexIdxA[1]==hexIdxB[1]+2 or hexIdxA[1]==hexIdxB[1]-2:
                if hexIdxA[0]==hexIdxB[0]:
                    return True
            elif hexIdxA[1] == hexIdxB[1]+1 or hexIdxA[1] == hexIdxB[1]-1:
                if hexIdxB[1]%2==0:
                    if hexIdxA[0] == hexIdxB[0] or hexIdxA[0] == hexIdxB[0]-1:
                        return True
                elif hexIdxB[1]%2==1:
                    if hexIdxA[0] == hexIdxB[0] or hexIdxA[0] == hexIdxB[0]+1:
                        return True
        
        return False 
    
    def getConnectedHexes(self, hexIdx):
        connectedHexes = []
        if self.orientation=="FLAT":
            directionList = FlatDirectionList
        elif self.orientation=="POINTY":
            directionList = PointyDirectionList
        for d in directionList:
            #print d
            nextHex = self.getNextHex(hexIdx, d)
            if nextHex != None:
                connectedHexes.append(nextHex)
        return connectedHexes
    
    def getHexesByRadius(self, hexIdx, radius):
        hexes = []
        if radius < 1:
            return hexes
        hexes.append(hexIdx)
        for r in range(radius):
            newHexes = []
            for hx in hexes:
                newHexes += self.getConnectedHexes(hx)
                
            for nhx in newHexes:
                if False == isHexInList(hexes, nhx):
                    hexes.append(nhx)
        return hexes
                    
    def getNextHex(self, hexIdx, direction):
        nextHexIdx = None
        
        if self.orientation=="FLAT":
            if direction=="U":
                if hexIdx[1] > 1:
                    nextHexIdx = [0,0]
                    nextHexIdx[0] = hexIdx[0]
                    nextHexIdx[1] = hexIdx[1]-2
                    return nextHexIdx                           
            elif direction=="D":            
                if hexIdx[1] < self.y_num-2:
                    nextHexIdx = [0,0]
                    nextHexIdx[0] = hexIdx[0]
                    nextHexIdx[1] = hexIdx[1]+2
                    return nextHexIdx                      
            elif direction=="SE":
                if hexIdx[0] < self.x_num-1 and hexIdx[1] < self.y_num-1:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]+1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0]
                    elif hexIdx[1]%2==1:                        
                        nextHexIdx[0] = hexIdx[0]+1 
                    return nextHexIdx            
            elif direction=="SW":
                if hexIdx[0] > 0 and hexIdx[1] < self.y_num-1:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]+1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0]-1
                    elif hexIdx[1]%2==1:                        
                        nextHexIdx[0] = hexIdx[0]                   
                    return nextHexIdx
            elif direction=="NE":
                if hexIdx[0] < self.x_num-1 and hexIdx[1] > 0:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]-1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0]
                    elif hexIdx[1]%2==1:                        
                        nextHexIdx[0] = hexIdx[0]+1 
                    return nextHexIdx            
            elif direction=="NW":
                if hexIdx[0] > 0 and hexIdx[1] > 0:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]-1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0]-1
                    elif hexIdx[1]%2==1:                        
                        nextHexIdx[0] = hexIdx[0] 
                    return nextHexIdx            
        elif self.orientation=="POINTY":
            if direction=="E":
                if hexIdx[0] < self.x_num-1:
                    nextHexIdx = [0,0]
                    nextHexIdx[0] = hexIdx[0]+1
                    nextHexIdx[1] = hexIdx[1]
                    return nextHexIdx                           
            elif direction=="W":            
                if hexIdx[0] > 0:
                    nextHexIdx = [0,0]
                    nextHexIdx[0] = hexIdx[0]-1
                    nextHexIdx[1] = hexIdx[1]
                    return nextHexIdx                      
            elif direction=="SE":
                if hexIdx[0] < self.x_num-1 and hexIdx[1] < self.y_num-1:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]+1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0] 
                    else:
                        nextHexIdx[0] = hexIdx[0]+1
                    return nextHexIdx            
            elif direction=="SW":
                if hexIdx[0] > 0 and hexIdx[1] < self.y_num-1:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]+1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0]-1 
                    else:
                        nextHexIdx[0] = hexIdx[0]
                    return nextHexIdx
            elif direction=="NE":
                if hexIdx[0] < self.x_num-1 and hexIdx[1] > 0:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]-1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0] 
                    else:
                        nextHexIdx[0] = hexIdx[0]+1
                    return nextHexIdx            
            elif direction=="NW":
                if hexIdx[0] > 0 and hexIdx[1] > 0:
                    nextHexIdx = [0,0]
                    nextHexIdx[1] = hexIdx[1]-1
                    if hexIdx[1]%2==0:
                        nextHexIdx[0] = hexIdx[0]-1 
                    else:
                        nextHexIdx[0] = hexIdx[0]
                    return nextHexIdx
        return None
    
    def generateTopologyGraph(self):
        
        g = TopologyGraph(False, 'Topo')
        for i in range(self.x_num):
            for j in range(self.y_num):
                g.addVertex([i,j])
        for i in range(self.x_num):
            for j in range(self.y_num):
                hxs = self.getConnectedHexes([i,j])
                #print str(len(hxs))
                for nh in hxs:
                    g.addEdge(([i,j],nh))
                    
        g.dump()
        
        return g
                                
            
                
        
            
        

        
        
        
        
        