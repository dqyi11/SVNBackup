from PyQt4 import QtGui, QtCore
from Agent import *
import pickle
import numpy as np

class HexagonalMapState(object):

    def __init__(self, map, valDim=1):
        self.map = map
        self.x_num = map.x_num
        self.y_num = map.y_num
        
        self.defaultEdgeColor = QtGui.QColor(0,0,0)
        self.defaultEdgeWidth = 1

        self.defaultActiveCellColor = QtGui.QColor(0, 255, 255)
        
        self.defaultObstacleColor = QtGui.QColor(255, 165, 0)
        self.defaultHumanColor = QtGui.QColor(0, 204, 0)
        self.defaultRobotColor = QtGui.QColor(0, 0, 204)
        
        self.refStartHexIdx = None
        self.refEndHexIdx = None
        self.refStartColor = QtGui.QColor(102, 102, 255)
        self.refEndColor = QtGui.QColor(153, 153, 0)
        
        self.activeCells = []
        self.currentHexValDim = 0
        self.hexValDim = valDim
        self.hexVals = []
        for d in range(self.hexValDim):
            hexVal = np.ones((self.x_num, self.y_num))
            self.hexVals.append(hexVal)
        
        self.accessible = np.ones((self.x_num, self.y_num))
        
        self.humanPath = []
        self.robotPath = []
        self.human = Agent()
        self.robot = Agent()
        
        #self.initRndVal()
        
        
    def clearActiveCells(self):
        self.activeCells = []
        
    def isActiveCell(self, cellIndex):
        for cell in self.activeCells:
            if cell[0]==cellIndex[0] and cell[1]==cellIndex[1]:
                return True
        return False
        
    def addActiveCells(self, cellIndex):
        if False==self.isActiveCell(cellIndex):
            self.activeCells.append(cellIndex)
            
    def initRndVal(self):
        for k in range(self.hexValDim):
            for i in range(self.x_num):
                rndVal = np.random.random(self.y_num)
                for j in range(self.y_num):
                    self.hexVals[k][i,j] = rndVal[j] 
                    
    def clearVal(self):
        self.hexVals = []
        for d in range(self.hexValDim):
            hexVal = np.ones((self.x_num, self.y_num))
            self.hexVals.append(hexVal)
                    
    def getHexColor(self, i, j, k=0):
        val = self.hexVals[k][i,j]
        int_val = int(255.0*val)
        #print str(int_val) + " : " + str(val)
        if int_val > 255.0:
            int_val = 255.0
        
        return QtGui.QColor(int_val, int_val, int_val)
    
    def dumpVal(self, filename):        
        pickle.dump( self.hexVals, open( filename, "wb" ) )
        
    def loadVal(self, filename):
        self.hexVals = pickle.load( open( filename, "rb" ) )
        self.hexValDim = len(self.hexVals)
        
    def loadFromArray(self, dataArray):
        for i in range(self.x_num):
            for j in range(self.y_num):
                self.hexVals[0][i,j] = dataArray[int(self.map.hexes[i][j].center[0]),int(self.map.hexes[i][j].center[1])]
        
        minVal = self.hexVals[0].min()
        maxVal = self.hexVals[0].max()
        ran = float(maxVal - minVal)
        for i in range(self.x_num):
            for j in range(self.y_num):
                self.hexVals[0][i,j] = (self.hexVals[0][i,j]-minVal)/ran
        
        
