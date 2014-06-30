#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
import numpy as np

class PlannerMonitor(QtGui.QWidget):
    
    def __init__(self, dim, scale):
        self.mapSize = dim
        self.scale = scale
        self.timerId = -1
        super(PlannerMonitor, self).__init__()
        self.initUI()
        self.planner = None
        
    def initUI(self):
        
        self.setGeometry(300, 300, self.mapSize[0], self.mapSize[1])
        self.show()
        
    def paintEvent(self, e):
        
        qp = QtGui.QPainter()
        qp.begin(self)
        
        blackColor = QtGui.QColor(0,0,0)
        greenColor = QtGui.QColor(0,255,0)
        redColor = QtGui.QColor(255,0,0)
        blueColor = QtGui.QColor(0,0,255)
        qp.setPen(blackColor)
        qp.setBrush(blackColor)        
        
        if self.planner != None:
            for t in range(len(self.planner.wayPoints)):
                p = self.planner.wayPoints[t]
                px, py = self.convertToDisplayCoordinate(p[0], p[1])
                if t < self.planner.tIdx:
                    qp.setBrush(greenColor)
                    qp.setPen(greenColor)
                else:
                    qp.setBrush(blackColor)
                    qp.setPen(blackColor)
                qp.drawEllipse(px-2.5,py-2.5,5,5)
            
            mx, my = self.convertToDisplayCoordinate(self.planner.currentPos['x'], self.planner.currentPos['y'])
            dmx, dmy = self.convertToDisplayScale(np.cos(self.planner.currentOrientation['yaw']), np.sin(self.planner.currentOrientation['yaw']))

            qp.setPen(blackColor)
            qp.setBrush(redColor)
            qp.drawEllipse(mx-5,my-5,10,10)
            qp.drawLine(mx,my,mx+dmx,my+dmy)
            
            
            if self.planner.tIdx < len(self.planner.wayPoints):
                qp.setPen(blueColor)
                wayX, wayY = self.convertToDisplayCoordinate(self.planner.wayPoints[self.planner.tIdx][0],self.planner.wayPoints[self.planner.tIdx][1])
                qp.drawLine(mx,my,wayX,wayY)
            
            #strvec = str(self.planner.currentOrientation['yaw'])+" : "+str(self.planner.currentOrientation['roll'])
            #qp.setFont(QtGui.QFont('Decorative',10))
            #qp.drawText(QtCore.QRect(mx,my,300,50), QtCore.Qt.AlignCenter, strvec)
        
        qp.end() 
        
    def convertToDisplayCoordinate(self, x, y):
        dispX = x*self.scale
        dispY = - y*self.scale
        dispX = dispX + self.mapSize[0]/2
        dispY = dispY + self.mapSize[1]/2
        return dispX, dispY
    
    def convertToDisplayScale(self, x, y):    
        dispX = x*self.scale
        dispY = - y*self.scale
        return dispX, dispY
        
    def startMonitor(self, time):
        self.timerId = self.startTimer(time*1000)
        
    def stopMonitor(self):
        self.killTimer(self.timerId)
        
    def timerEvent(self, e):
        #print e
        self.planner.update()
        self.update()
        self.planner.control()
