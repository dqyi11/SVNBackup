from PyQt4 import QtGui, QtCore
from MapViewConfigForm import *
from HexagonalMap import *
from HexaMapWidget import *
from HexaUtils import *
from PlanningPathGenerator import *
from TreeExpandingPathPlanner import *
import copy

class MapViewForm(QtGui.QMainWindow):
    
    def __init__(self, size):
        super(QtGui.QMainWindow, self).__init__()
        self.formSize = size
        self.resize(self.formSize[0], self.formSize[1])
        
        self.hexSize = 10
        self.hexOrientation = "POINTY"
        self.obsThreshold = 60
        self.considerObstacle = True
        
        self.wingmanRadius = 2
        self.humanObsR = 2
        self.robotObsR = 2
        self.humanDiscountFactor = 0.4
        self.robotDiscountFactor = 0.4
        
        self.hexaMap = None
        self.planningPathGenerator = None
        
        self.configWindow = MapViewConfigForm(self)
        self.initUI()
        self.cursorPos = None
        
    def initUI(self):        
        openAction = QtGui.QAction('Open', self)
        openAction.triggered.connect(self.openMap)
        configAction = QtGui.QAction('Config', self)
        configAction.triggered.connect(self.configParam)
        loadDataAction = QtGui.QAction('Load', self)
        loadDataAction.triggered.connect(self.loadData)
        saveDataAction = QtGui.QAction('Save', self)
        saveDataAction.triggered.connect(self.saveData)
                
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(openAction)
        toolMenu = menubar.addMenu('&Tool')
        toolMenu.addAction(configAction)
        toolMenu.addAction(saveDataAction)
        toolMenu.addAction(loadDataAction)
    
        self.mapView = QtGui.QLabel()
        self.setCentralWidget(self.mapView)        
        self.show()
               
    def configParam(self):
        self.configWindow.show()
        
    def openMap(self):
        
        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file')
        pixmap = QtGui.QPixmap(fname)
        
        self.mapView.setPixmap(pixmap)
        
        self.formSize[0] = pixmap.width()
        self.formSize[1] = pixmap.height()
        
        self.mapView.resize(self.formSize[0], self.formSize[1])
        self.imageData = pixmap.toImage()
        self.setCentralWidget(self.mapView) 
        
        self.hexaMap = None
        self.planningPathGenerator = None
        
        self.update()
        
    def saveData(self):
        fname = QtGui.QFileDialog.getSaveFileName(self, 'save file')
        if fname!= None:
            self.hexaMap.hexamapState.dumpVal(fname)
            
    def loadData(self):
        fname = QtGui.QFileDialog.getOpenFileName(self, 'open file')
        if fname!=None:
            self.hexaMap.hexamapState.loadVal(fname)
            self.update()
        
    def generateHexaMap(self):        
        self.x_num, self.y_num = calcHexDimension(self.formSize[0], self.formSize[1], self.hexSize, self.hexOrientation)
        self.hexaMap = HexaMapWidget(self.x_num, self.y_num, self.hexSize, self.hexOrientation)
        self.setCentralWidget(self.hexaMap)
        if self.considerObstacle == True:
            self.updateObstalce()
        self.planningPathGenerator = PlanningPathGenerator(self.hexaMap.hexamap)
        #self.hexaMap.hexamap.generateTopologyGraph()
        
        self.update() 
        
    def updateObstalce(self):        
        darkCnt = np.zeros((self.x_num, self.y_num))
        for i in range(self.formSize[0]):
            for j in range(self.formSize[1]):
                colorVal = self.getDataVal(i, j)
                if colorVal < self.obsThreshold:
                    idx = self.hexaMap.hexamap.findHex(i,j)
                    darkCnt[idx[0], idx[1]]+=1
                    
        for i in range(self.x_num):
            for j in range(self.y_num):
                if darkCnt[i,j] >= 4:
                    self.hexaMap.hexamapState.accessible[i,j] = 0
                else:
                    self.hexaMap.hexamapState.accessible[i,j] = 1
        
    def getDataVal(self, x, y):
        int_x = int(x)
        int_y = int(y)        
        p = self.imageData.pixel(int_x, int_y)
        return QtGui.qGray(p)
    
    def mousePressEvent(self, e):
        if e.button() == QtCore.Qt.RightButton:
            if self.hexaMap != None:
                x_pos = e.pos().x()
                y_pos = e.pos().y()
                hexIdx = self.hexaMap.hexamap.findHex(x_pos, y_pos)
                if hexIdx != None:
                    self.hexaMap.hexamapState.humanPath = []
                    self.hexaMap.hexamapState.humanPath.append(hexIdx)
                    self.update()
    
    def keyPressEvent(self, event):
        if type(event) == QtGui.QKeyEvent:
            humanPathLen = len(self.hexaMap.hexamapState.humanPath) 
            if humanPathLen < 1:
                return
            nextDirection = None           
            if event.key() == QtCore.Qt.Key_W:
                nextDirection = "NW"
            elif event.key() == QtCore.Qt.Key_E:
                nextDirection = "NE"
            elif event.key() == QtCore.Qt.Key_A:
                nextDirection = "W"
            elif event.key() == QtCore.Qt.Key_D:
                nextDirection = "E"
            elif event.key() == QtCore.Qt.Key_Z:
                nextDirection = "SW"
            elif event.key() == QtCore.Qt.Key_X:
                nextDirection = "SE"
            print nextDirection
            if nextDirection != None:
                nextHex = self.hexaMap.hexamap.getNextHex(self.hexaMap.hexamapState.humanPath[humanPathLen-1], nextDirection)
                self.hexaMap.hexamapState.humanPath.append(nextHex)
                self.update()
            event.accept()
        else:
            event.ignore()
            
    def planPath(self):        
        plannedPathGraph = self.planningPathGenerator.generatePlanningPathGraph(self.hexaMap.hexamapState.humanPath, self.wingmanRadius, "planningPath")

        #plannedPathGraph.printString()
        plannedPathGraph = self.planningPathGenerator.backwardPrune(plannedPathGraph)
        plannedPathGraph = self.planningPathGenerator.forwardPrune(plannedPathGraph)
        
        #plannedPathGraph.dump()
        
        rewardDistribution = copy.deepcopy(self.hexaMap.hexamapState.hexVals[0])
        humanPath = self.hexaMap.hexamapState.humanPath
        planningLen = len(humanPath)
        planner = TreeExpandingPathPlanner(self.hexaMap.hexamap, self.hexaMap.hexamapState.robot)
        self.hexaMap.hexamapState.robotPath = planner.planPath(plannedPathGraph, humanPath[0], planningLen, rewardDistribution)
        
        print planner.iterationCount        
        
        self.update()
        
        
    def applyHumanPath(self):
        if self.hexaMap != None:
            self.hexaMap.hexamapState.human.discountFactor = self.humanDiscountFactor
            self.hexaMap.hexamapState.robot.discountFactor = self.robotDiscountFactor
            self.hexaMap.hexamapState.human.observeRange = self.humanObsR
            self.hexaMap.hexamapState.robot.observeRange = self.robotObsR
            
            rewardDistribution = copy.deepcopy(self.hexaMap.hexamapState.hexVals[0])
            humanPath = self.hexaMap.hexamapState.humanPath
            human = self.hexaMap.hexamapState.human
            for hx in humanPath:
                human.pos = hx
                rewardDistribution = human.applyObservation(hx, self.hexaMap.hexamap, rewardDistribution)
            self.hexaMap.hexamapState.hexVals[0] = rewardDistribution    
            #print rewardDistribution
            self.update()
            