'''
Created on Apr 22, 2014

@author: walter
'''

from PyQt4 import QtGui, QtCore
import math

class MapViewer(QtGui.QLabel):
    '''
    classdocs
    '''

    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)
        self.mPoint = QtCore.QPoint(5,5)
        self.mPointYaw = 0.0;
        self.mShowPoint = False
        self.plannedPath = None
        self.plannedPathReachedIdx = 0
    
        
    def paintEvent(self, e):
        
        super(MapViewer, self).paintEvent(e)
        
        if self.mShowPoint == True:
            qp = QtGui.QPainter()
            qp.begin(self)
            qp.setPen(QtGui.QPen(QtGui.QColor(255,0,0)))
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255,0,0)))
            qp.drawEllipse(self.mPoint, 5, 5)
            orientX = self.mPoint.x() + 10.0 * math.cos(self.mPointYaw)
            orientY = self.mPoint.y() + 10.0 * math.sin(self.mPointYaw)
            qp.setPen(QtGui.QPen(QtGui.QColor(0,0,255)))
            qp.drawLine(self.mPoint, QtCore.QPoint(orientX, orientY))
            
            if self.plannedPath != None:
                
                for t in range(self.plannedPath.length):
                    pt = self.plannedPath.waypoints[t]
                    if t < self.plannedPathReachedIdx:
                        qp.setPen(QtGui.QPen(QtGui.QColor(0,0,255)))
                        qp.setBrush(QtGui.QBrush(QtGui.QColor(0,0,255)))
                    else:
                        qp.setPen(QtGui.QPen(QtGui.QColor(0,255,0)))
                        qp.setBrush(QtGui.QBrush(QtGui.QColor(0,255,0)))
                    qp.drawEllipse(QtCore.QPoint(int(pt[0]), int(pt[1])), 3, 3)
                    
                qp.setPen(QtGui.QPen(QtGui.QColor(0,0,0), 1, QtCore.Qt.DashLine))
                
                if self.plannedPathReachedIdx < self.plannedPath.length:
                    targetPoint = self.plannedPath.waypoints[self.plannedPathReachedIdx]
                    targetX = int(targetPoint[0])
                    targetY = int(targetPoint[1])                
                    qp.drawLine(self.mPoint, QtCore.QPoint(targetX, targetY))
            
            qp.end()
            
    def setPoint(self,x,y):
        self.mPoint.setX(x)
        self.mPoint.setY(y)
        
    
            
        