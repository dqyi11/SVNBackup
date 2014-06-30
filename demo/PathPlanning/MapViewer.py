from PyQt4 import QtGui, QtCore
from MapInfo import *

class MapViewer(QtGui.QLabel):

    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)
        self.states = {'normal':0,'addingModal':1}
        self.state = self.states['normal']
        self.addingEnvModal = False
        self.modalRadiusX = 0
        self.modalRadiusY = 0
        self.modalCenterX = 0
        self.modalCenterY = 0
        self.multiModalEnv = MultiModalEnv()
        
    def initMapInfo(self):
        self.mapInfo = MapInfo(self)
        
    def mouseMoveEvent(self,event):
        if self.state == self.states['addingModal']:
            if self.addingEnvModal == True:
                #print "mouse move"
                currentX = event.x()
                currentY = event.y()
                self.modalRadiusX = currentX-self.modalCenterX
                self.modalRadiusY = currentY-self.modalCenterY
                #print self.modalRadius
                self.update()
            

    def mousePressEvent(self, QMouseEvent):
        #print "move"
        if self.state == self.states['normal']:
            if QMouseEvent.button()==QtCore.Qt.LeftButton:
                self.state = self.states['addingModal']
                self.addingEnvModal = True
                self.modalCenterX = QMouseEvent.x()
                self.modalCenterY = QMouseEvent.y()
        elif self.state == self.states['addingModal']:
            if QMouseEvent.button()==QtCore.Qt.LeftButton:
                if self.addingEnvModal == True:
                    self.addingEnvModal = False
                    self.multiModalEnv.addModal([self.modalCenterX, self.modalCenterY],[self.modalRadiusX, self.modalRadiusY])
                else:
                    self.addingEnvModal = True
                    self.modalCenterX = QMouseEvent.x()
                    self.modalCenterY = QMouseEvent.y()                                       
            elif QMouseEvent.button()==QtCore.Qt.RightButton:
                reply = QtGui.QMessageBox.question(self, 'Exit', 'Apply modals?', QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
                if reply==QtGui.QMessageBox.Yes:
                    self.geneMMEnv()
                    self.updateEnv()
                    self.update()
                    self.multiModalEnv.clear()
                self.state = self.states['normal']
                self.addingEnvModal = False
                
        
    def paintEvent(self, e):
        super(MapViewer, self).paintEvent(e)
        
        if self.state == self.states['addingModal']:
            if self.addingEnvModal == True:
                #print "x:"+str(self.modalCenterX)+",Y:"+str(self.modalCenterY)+" "+str(self.modalRadius)
                qp = QtGui.QPainter()
                qp.begin(self)
                pen = QtGui.QPen(QtGui.QColor(0,0,255), QtCore.Qt.SolidLine)
                pen.setStyle(QtCore.Qt.DashLine)
                qp.setPen(pen)
                centerPoint = QtCore.QPoint(self.modalCenterX, self.modalCenterY)
                qp.drawPoint(centerPoint)
                qp.drawEllipse(centerPoint, self.modalRadiusX, self.modalRadiusY)
                qp.end()
        
        qp = QtGui.QPainter()
        qp.begin(self)
        pen = QtGui.QPen(QtGui.QColor(0,0,255))
        qp.setPen(pen)        
        for i in range(len(self.multiModalEnv.centers)):
            cet = self.multiModalEnv.centers[i]
            spd = self.multiModalEnv.spreadings[i]
            qp.drawEllipse(QtCore.QPoint(cet[0],cet[1]), spd[0], spd[1])
        qp.end()    
            
                
    def updateEnv(self):
            
        pixmap = self.pixmap()
        qp = QtGui.QPainter(pixmap)
        #qp.begin(self)
        for i in range(pixmap.width()):
            for j in range(pixmap.height()):
                if self.mapInfo.pointAcc[i,j] == True:
                    r,g,b = self.mapInfo.getColor(i,j)
                    qp.setPen(QtGui.QPen(QtGui.QColor(r,g,b)))
                    qp.drawPoint(i,j)
                    
        qp.end()
        self.update()
                
    def geneRndEnv(self):
        for i in range(self.width()):
            for j in range(self.height()):
                self.mapInfo.pointVal[i,j] = np.random.rand()
                
    def geneMMEnv(self):
        for i in range(self.width()):
            for j in range(self.height()):
                self.mapInfo.pointVal[i,j] = self.multiModalEnv.calc([i,j])
        
        
class MultiModalEnv(object):
    
    def __init__(self):        
        self.centers = []
        self.spreadings = []
        
    def addModal(self, center, spreading):
        self.centers.append(center)
        self.spreadings.append(spreading)
        
    def clear(self):
        self.centers = []
        self.spreadings = []
        
    def calc(self, pos):
        val = 0.0
        for i in range(len(self.centers)):
            dX = float(pos[0])-float(self.centers[i][0])
            dY = float(pos[1])-float(self.centers[i][1])
            sigmaXSquare=float(self.spreadings[i][0])**2
            sigmaYSquare=float(self.spreadings[i][1])**2
            exponent = -0.5
            sum = dX**2/sigmaXSquare+dY**2/sigmaYSquare
            exponent *= sum
            newVal = math.exp(exponent)
            if val < newVal:
                val = newVal
            
        return val            
        
        
        
        
        
    
    
            
        