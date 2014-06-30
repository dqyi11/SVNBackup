from PyQt4 import QtGui, QtCore
import math
import numpy as np
from interparc import interparc
from WaypointMAS import *

class MapViewer(QtGui.QLabel):
    
    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)
        self.recordingPos = False
        self.points = []
        self.newpoints = []
        self.sampleSize = 0
        self.pointsCurveLength = 0.0
        self.sampleInterval = 60.0
        self.toleranceRegionSize = [120, 120]
        
        self.startPos = []
        self.endPos = []
        
        self.particleNum = 4
        self.mas = WaypointMultiAgentSystem(self.particleNum)
        
        self.RVals = np.random.randint(0,255,self.particleNum)
        self.GVals = np.random.randint(0,255,self.particleNum)
        self.BVals = np.random.randint(0,255,self.particleNum)
        
        
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
        
        qp.setPen(QtGui.QPen(QtGui.QColor(200,200,200)))
        qp.setBrush(QtGui.QBrush(QtGui.QColor(200,200,200)))
        for p in self.points:
            qp.drawEllipse(QtCore.QPoint(p[0],p[1]), 1, 1)
            
        qp.setPen(QtGui.QPen(QtGui.QColor(0,0,255)))
        qp.setBrush(QtGui.QBrush(QtGui.QColor(0,0,255))) 
        for newpoint in self.newpoints:
            qp.drawRect(newpoint[0],newpoint[1], 3, 3)  
            
        qp.setPen(QtGui.QPen(QtGui.QColor(0,255,255), 1, QtCore.Qt.DashLine))
        qp.setBrush(QtCore.Qt.NoBrush)
        for agent in self.mas.agents:
            #print agent.searchRegion
            qp.drawRect(int(agent.searchRegion[0][0]), int(agent.searchRegion[1][0]), self.toleranceRegionSize[0], self.toleranceRegionSize[1])
        

        
        for i in range(self.particleNum):
            for j in range(len(self.mas.agents)):
                curr_agent = self.mas.agents[j]
                curr_par = curr_agent.particles[i]
                p_color = QtGui.QColor(self.RVals[i], self.GVals[i], self.BVals[i])
                qp.setPen(QtGui.QPen(p_color))
                qp.setBrush(QtGui.QBrush(p_color))
                pointPos = QtCore.QPoint(int(curr_par.pos[0]), int(curr_par.pos[1]))
                qp.drawEllipse(pointPos, 2,2)
                if j < len(self.mas.agents)-1:
                    next_agent = self.mas.agents[j+1]
                    next_par = next_agent.particles[i]
                    qp.setPen(QtGui.QPen(p_color, 1, QtCore.Qt.DotLine))
                    nextPointPos = QtCore.QPoint(int(next_par.pos[0]),int(next_par.pos[1]))
                    qp.drawLine(pointPos, nextPointPos)
        
        qp.end()
        
    def mousePressEvent(self, e):
        
        if e.button() == QtCore.Qt.LeftButton:
            if len(self.startPos) == 2 and len(self.endPos)==2:
                self.recordingPos = True

            
    def mouseReleaseEvent(self, e):
        
        if e.button() == QtCore.Qt.LeftButton:
            
            if self.recordingPos == True:
                self.recordingPos = False
                self.resample()

                self.update()
        
    def mouseMoveEvent(self, e):
        
        if self.recordingPos == True:
            point = [ e.pos().x(), e.pos().y() ]
            self.points.append(point)
            self.update()
            
    def calcPointCurveLength(self):
        
        self.pointsCurveLength = 0.0
        for i in range(len(self.points)-1):
            p1 = self.points[i]
            p2 = self.points[i+1]
            deltaDist = np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
            self.pointsCurveLength += deltaDist
            
        return self.pointsCurveLength
        
            
    def resample(self):
        xs = []
        ys = []
        xs.append(self.startPos[0])
        ys.append(self.startPos[1])
        for pt in self.points:
            xs.append(pt[0])
            ys.append(pt[1])
        xs.append(self.endPos[0])
        ys.append(self.endPos[1])
            
        self.calcPointCurveLength()
        
        print self.pointsCurveLength
        self.sampleSize = int(self.pointsCurveLength/self.sampleInterval)
            
        newpts = interparc(self.sampleSize, xs, ys)
        
        self.newpoints = []
        for npt in newpts:
            self.newpoints.append([int(npt[0]), int(npt[1])])
            
        self.mas.agents = []
        searchRegionSize = [float(self.toleranceRegionSize[0]), float(self.toleranceRegionSize[1])]
        for np in self.newpoints:
            centerPos = [float(np[0]), float(np[1])]
            self.mas.addAgent(centerPos, searchRegionSize)
            
        self.points = []
        
        #self.mas.update()
            
    def dump(self):
        
        dataLen = len(self.points)
        xs = np.zeros(dataLen, dtype=np.int)
        ys = np.zeros(dataLen, dtype=np.int)
        for t in range(dataLen):
            xs[t] = self.points[t][0]
            ys[t] = self.points[t][1]
        
        np.savetxt('x.csv', xs, delimiter=',', fmt="%d")
        np.savetxt('y.csv', ys, delimiter=',', fmt="%d")
        
    def addStart(self, pos):
        self.startPos = [pos.x(), pos.y()]
        self.points = []
        self.newpoints = []
        self.mas.agents = []
        self.update()
        
    def addEnd(self, pos):
        self.endPos = [pos.x(), pos.y()]
        self.points = []
        self.newpoints = []
        self.mas.agents = []
        self.update()
        
        