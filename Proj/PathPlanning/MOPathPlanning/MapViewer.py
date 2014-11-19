from PyQt4 import QtGui, QtCore
import math
import numpy as np

class MapViewer(QtGui.QLabel):
    
    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)        
        self.startPos = []
        self.endPos = []
        self.pathList = []
                
    def paintEvent(self, e):
        super(MapViewer, self).paintEvent(e)
        qp = QtGui.QPainter()
        qp.begin(self)
        
        if len(self.startPos) == 2:
            inside_size = 4
            outside_size = 8      
            delta_size = (outside_size-inside_size)/2
            qp.setPen(QtGui.QPen(QtGui.QColor(255,255,0)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,255,0))) 
            qp.drawRect(self.startPos[0]-delta_size, self.startPos[1]-delta_size, outside_size, outside_size)
            qp.setPen(QtGui.QPen(QtGui.QColor(255,0,0)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,0,0))) 
            qp.drawRect(self.startPos[0], self.startPos[1], inside_size, inside_size)
        
        if len(self.endPos)==2:
            inside_size = 4
            outside_size = 8 
            delta_size = (outside_size-inside_size)/2
            qp.setPen(QtGui.QPen(QtGui.QColor(255,255,0)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,255,0))) 
            qp.drawRect(self.endPos[0]-delta_size, self.endPos[1]-delta_size, outside_size, outside_size)
            qp.setPen(QtGui.QPen(QtGui.QColor(255,0,255)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,0,255)))
            qp.drawRect(self.endPos[0], self.endPos[1], inside_size, inside_size)
            
        for path in self.pathList:
            qp.setPen(QtGui.QPen(QtGui.QColor(204,0,204)))
            qp.drawLine(self.startPos[0], self.startPos[1], path[0][0], path[0][1])
            for i in range(len(path)-1):
                qp.drawLine(path[i][0],path[i][1],path[i+1][0],path[i+1][1])
            qp.drawLine(path[len(path)-1][0], path[len(path)-1][1], self.endPos[0], self.endPos[1])
        
        qp.end()
            
    def addStart(self, pos):
        self.startPos = [pos.x(), pos.y()]
        self.update()
        
    def addEnd(self, pos):
        self.endPos = [pos.x(), pos.y()]
        self.update()
        
        