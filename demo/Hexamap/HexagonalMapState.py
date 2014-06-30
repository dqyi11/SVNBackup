from PyQt4 import QtGui, QtCore
import numpy as np

class HexagonalMapState(object):

    def __init__(self, map, valDim=1):
        self.map = map
        self.x_num = map.x_num
        self.y_num = map.y_num
        
        self.defaultEdgeColor = QtGui.QColor(0,0,0)
        self.defaultEdgeWidth = 1

        self.defaultActiveCellColor = QtGui.QColor(0,255,255)
        
        self.activeCells = []
        self.hexValDim = valDim
        self.hexVals = np.ones((self.x_num, self.y_num, self.hexValDim))
        
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
                    self.hexVals[i,j,k] = rndVal[j] 
                    
    def getHexColor(self, i, j, k=0):
        val = self.hexVals[i,j,k]
        int_val = int(255*val)
        if int_val > 255:
            int_val = 255
        #print int_val
        return QtGui.QColor(int_val, int_val, int_val)
        
        
        
