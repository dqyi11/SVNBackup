from PyQt4 import QtGui, QtCore
import math
import numpy as np

class MapViewer(QtGui.QLabel):
    
    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)        
        self.startPos = []
        self.endPos = []
                
    def paintEvent(self, e):
        super(MapViewer, self).paintEvent(e)
        qp = QtGui.QPainter()
        qp.begin(self)
        
        if len(self.startPos) == 2:
            qp.setPen(QtGui.QPen(QtGui.QColor(255,0,0)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,0,0)))          
            qp.drawRect(self.startPos[0], self.startPos[1], 4, 4)
        
        if len(self.endPos)==2:
            qp.setPen(QtGui.QPen(QtGui.QColor(255,0,255)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,0,255)))
            qp.drawRect(self.endPos[0], self.endPos[1], 4, 4)
        
        qp.end()
            
    def addStart(self, pos):
        self.startPos = [pos.x(), pos.y()]
        self.update()
        
    def addEnd(self, pos):
        self.endPos = [pos.x(), pos.y()]
        self.update()
        
        