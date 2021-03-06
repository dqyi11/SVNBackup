from PyQt4 import QtGui, QtCore
from MapViewConfigForm import *
from HexagonalMap import *
from HexaMapWidget import *
from HexaUtils import *
from PlanningPathGenerator import *
from TreeExpandingPathPlanner import *
from LabelManager import *
from VisibilityDataMgr import *
from InfoDistributionGenerator import *
from PathManager import *
from PIL import Image
import copy
import os

class MapViewForm(QtGui.QMainWindow):
    
    def __init__(self, size):
        super(QtGui.QMainWindow, self).__init__()
        self.formSize = size
        self.resize(self.formSize[0], self.formSize[1])
        
        self.hexSize = 5
        self.hexOrientation = "POINTY"
        self.obsThreshold = 60
        self.considerObstacle = True
        self.dataDim = 1
        
        self.referenceStates = ["Manual", "Quickly", "Safely"]
        self.currentRefState = "Manual"
        
        self.planStates = ["InfoMax", "RiskMin"]
        self.currentPlanState = "InfoMax"
        
        self.dataViewStates = ["Infomation", "EnemyVisibilty", "PosVisibility","AverageVisibility"]
        self.currentDataViewState = "Infomation"
        
        self.wingmanRadius = 2
        self.humanObsR = 0
        self.robotObsR = 0
        self.humanDiscountFactor = 0.4
        self.robotDiscountFactor = 0.4
        
        self.hexaMap = None
        self.planningPathGenerator = None
        
        self.nondominatedSolutionMgr = None
        
        self.labelMgr = LabelManager()
        self.configWindow = MapViewConfigForm(self)
        self.initUI()
        self.cursorHexIdx = None
        
        self.visbilityDataMgr = VisibilityDataMgr()
        self.enemyVisibility = None
        
        self.diff = None
        self.mapFilename = ''
        
        
    def initUI(self):        
        openAction = QtGui.QAction('Open', self)
        openAction.triggered.connect(self.openMap)
        exportAction = QtGui.QAction('Export path', self)
        exportAction.triggered.connect(self.exportPath)        
        configAction = QtGui.QAction('Config', self)
        configAction.triggered.connect(self.configParam)
        loadDataAction = QtGui.QAction('Load', self)
        loadDataAction.triggered.connect(self.loadData)
        saveDataAction = QtGui.QAction('Save', self)
        saveDataAction.triggered.connect(self.saveData)
        importDiffAction = QtGui.QAction('Import Diffusion', self)
        importDiffAction.triggered.connect(self.importDiffusion)
        importVisAction = QtGui.QAction('Import Visibility', self)
        importVisAction.triggered.connect(self.importVisibility)
        clearDataAction = QtGui.QAction('Clear', self)
        clearDataAction.triggered.connect(self.clearEnvData)
        randomDataAction = QtGui.QAction('Random', self)
        randomDataAction.triggered.connect(self.randomEnvData)        
                
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(openAction)
        fileMenu.addAction(exportAction)
        toolMenu = menubar.addMenu('&Tool')
        toolMenu.addAction(configAction)
        toolMenu.addAction(saveDataAction)
        toolMenu.addAction(loadDataAction)
        toolMenu.addAction(importDiffAction)
        toolMenu.addAction(importVisAction)
        envMenu = menubar.addMenu('&Test')
        envMenu.addAction(clearDataAction)
        envMenu.addAction(randomDataAction)
        
        self.initContextMenu()
               
        self.show()
               
    def configParam(self):
        self.configWindow.show()
        
    def openMap(self):
        
        self.mapView = QtGui.QLabel()
        self.setCentralWidget(self.mapView) 
        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file')
                
        self.labelMgr.loadFile(fname)
        
        self.dirname = os.path.dirname(os.path.abspath(fname))
        
        pixmap = QtGui.QPixmap(self.dirname + "/" + self.labelMgr.mapFile)
        
        self.mapView.setPixmap(pixmap)
        
        self.formSize[0] = pixmap.width()
        self.formSize[1] = pixmap.height()
        
        #self.diffGnr = InfoDistributionGenerator(self.labelMgr, 500)
        #self.diffGnr.generateDistribution(self.labelMgr.obstacleFile)
        
        self.mapView.resize(self.formSize[0], self.formSize[1])
        self.imageData = pixmap.toImage()
        self.setCentralWidget(self.mapView) 
        
        self.hexaMap = None
        self.planningPathGenerator = None
        
        self.update()
        
    def saveData(self):
        fname = QtGui.QFileDialog.getSaveFileName(self, 'save file')
        if fname!= None and fname!='':
            self.hexaMap.hexamapState.dumpVal(fname)
            
    def loadData(self):
        fname = QtGui.QFileDialog.getOpenFileName(self, 'open file')
        if fname!=None and fname!='':
            self.hexaMap.hexamapState.loadVal(fname)
            self.update()
        
    def generateHexaMap(self):        
        self.x_num, self.y_num = calcHexDimension(self.formSize[0], self.formSize[1], self.hexSize, self.hexOrientation)
        self.hexaMap = HexaMapWidget(self.x_num, self.y_num, self.hexSize, self.hexOrientation, self.dataDim)
        self.setCentralWidget(self.hexaMap)
        if self.considerObstacle == True:
            self.updateObstalce()
        self.planningPathGenerator = PlanningPathGenerator(self.hexaMap.hexamap)
        #self.hexaMap.hexamap.generateTopologyGraph()
        self.configWindow.initCbInfoVec()
        self.configWindow.update()
        
        #self.visbilityDataMgr.randInit(self.x_num, self.y_num, self.hexSize)
        #self.visbilityDataMgr.dumpData('visData2.txt')
        
        if self.currentPlanState == self.planStates[0]:
            self.hexaMap.hexamapState.loadFromArray(self.diff)
            
        enemyHexIds = self.translatePosesToHexIds(self.labelMgr.getEnemyPos())
        self.generateEnemyVisibility(enemyHexIds)
        
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
                    if self.currentRefState == self.referenceStates[0]:
                        self.hexaMap.hexamapState.humanPath = []
                        self.hexaMap.hexamapState.humanPath.append(hexIdx)
                        self.update()
                    else:
                        self.showContextMenu(e.pos(), hexIdx)
        elif e.button() == QtCore.Qt.LeftButton:
            if self.hexaMap != None:
                x_pos = e.pos().x()
                y_pos = e.pos().y()
                hexIdx = self.hexaMap.hexamap.findHex(x_pos, y_pos)
                if hexIdx != None:
                    if self.currentDataViewState == self.dataViewStates[2]:
                    #if self.currentPlanState == self.planStates[1]:
                        self.visbilityDataMgr.currentHexId = hexIdx
                        for i in range(self.hexaMap.hexamap.x_num):
                            for j in range(self.hexaMap.hexamap.y_num):
                                self.hexaMap.hexamapState.hexVals[0][i,j] = self.visbilityDataMgr.getValue(i,j)
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
        
        humanPath = self.hexaMap.hexamapState.humanPath
        planningLen = len(humanPath)
        
        rewardDistribution = copy.deepcopy(self.hexaMap.hexamapState.hexVals[0])
        planner = TreeExpandingPathPlanner(self.hexaMap.hexamap, self.hexaMap.hexamapState.robot)
        planner.runOnlyOnce = True
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
            
    def clearEnvData(self):
        if self.hexaMap != None:
            self.hexaMap.hexamapState.clearVal()
            self.update()
    
    def randomEnvData(self):
        if self.hexaMap != None:
            self.hexaMap.hexamapState.initRndVal()
            self.update()
            
    def initContextMenu(self):

        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)  
        #self.customContextMenuRequested.connect(self.showContextMenu) 
        
        self.contextMenu = QtGui.QMenu(self)  
        self.actionAddStart = self.contextMenu.addAction('start')
        self.actionAddEnd = self.contextMenu.addAction('end')
        
        self.actionAddStart.triggered.connect(self.addStart)
        self.actionAddEnd.triggered.connect(self.addEnd)
        
    def showContextMenu(self, pos, hexIdx):
        
        self.contextMenu.move(self.pos() + pos)
        self.cursorHexIdx = hexIdx
        self.contextMenu.show()
        
    def addStart(self):
        if self.hexaMap != None:
            self.hexaMap.hexamapState.refStartHexIdx = self.cursorHexIdx
            self.update()
        
    def addEnd(self):
        if self.hexaMap != None:
            self.hexaMap.hexamapState.refEndHexIdx = self.cursorHexIdx
            self.update()
            
    def clearRefPath(self):
        
        if self.hexaMap != None:
            self.hexaMap.hexamapState.refStartHexIdx = None
            self.hexaMap.hexamapState.refEndHexIdx = None
            self.hexaMap.hexamapState.humanPath = []
            
    def genReference(self):
        
        if self.hexaMap != None:
            if self.currentRefState == self.referenceStates[1]:
                if self.hexaMap.hexamapState.refStartHexIdx != None and self.hexaMap.hexamapState.refEndHexIdx != None:
                    topograph = self.hexaMap.hexamap.generateTopologyGraph()
                    self.hexaMap.hexamapState.humanPath = topograph.findShortestPath(self.hexaMap.hexamapState.refStartHexIdx, self.hexaMap.hexamapState.refEndHexIdx)
                    self.hexaMap.hexamapState.refStartHexIdx = None
                    self.hexaMap.hexamapState.refEndHexIdx = None
                    self.update()
            elif self.currentRefState == self.referenceStates[2]:
                if self.hexaMap.hexamapState.refStartHexIdx != None and self.hexaMap.hexamapState.refEndHexIdx != None:
                    topograph = self.hexaMap.hexamap.generateTopologyGraph()
                    self.hexaMap.hexamapState.humanPath = topograph.findLeastRiskyPath(self.hexaMap.hexamapState.refStartHexIdx, self.hexaMap.hexamapState.refEndHexIdx,self.enemyVisibility)
                    self.hexaMap.hexamapState.refStartHexIdx = None
                    self.hexaMap.hexamapState.refEndHexIdx = None
                    self.update()
                    
    def importVisibility(self):
        fname = QtGui.QFileDialog.getOpenFileName(self, 'open file')
        if fname!=None and fname!='':
            self.visbilityDataMgr.loadFile(fname)  
            
    def importDiffusion(self):
        fname = QtGui.QFileDialog.getOpenFileName(self, 'open file')
        if fname!=None and fname!='':
            w = self.formSize[0]
            h = self.formSize[1]
            self.diff = np.zeros((w, h))
            print fname
            
            img = Image.open(str(fname))
            img.convert("L")
            for i in range(w):
                for j in range(h):
                    self.diff[i,j] = img.getpixel((i,j))
                    
    def exportPath(self):
        if len(self.hexaMap.hexamapState.robotPath) > 0:
            filename = QtGui.QFileDialog.getSaveFileName(self, "Save file", "", ".xml")
            pathMgr = PathManager(self.hexaMap.hexamap, self.labelMgr.mapFile, 5, self.formSize[0], self.formSize[1])
            pathMgr.dumpPath(self.hexaMap.hexamapState.robotPath, filename)
        
    def updateCurrentPlanState(self, state):
        self.currentPlanState = state
        
    def updateCurrentView(self):
        
        if self.hexaMap == None:
            return
        
        print self.currentDataViewState
        
        if self.currentDataViewState == self.dataViewStates[0]:
            self.hexaMap.hexamapState.loadFromArray(self.diff)
            self.update()
        elif self.currentDataViewState == self.dataViewStates[1]:
            self.hexaMap.hexamapState.hexVals[0] = copy.deepcopy(self.enemyVisibility)
            self.update()      
        elif self.currentDataViewState == self.dataViewStates[2]:
            #self.visbilityDataMgr.randInit(self.x_num, self.y_num, self.hexSize)
            self.visbilityDataMgr.currentHexId = None
            for i in range(self.hexaMap.hexamap.x_num):
                for j in range(self.hexaMap.hexamap.y_num):
                    self.hexaMap.hexamapState.hexVals[0][i,j] = 1.0
            self.update()
        elif self.currentDataViewState == self.dataViewStates[3]:
            self.hexaMap.hexamapState.hexVals[0] = copy.deepcopy(self.visbilityDataMgr.visSumData)
            self.update()
               
        
    def translatePosesToHexIds(self, poses):
        hexIds = []
        for pos in poses:
            hexId =  self.hexaMap.hexamap.findHex(pos[0], pos[1])
            if hexId != None:
                hexIds.append(hexId)
        return hexIds
    
    def generateEnemyVisibility(self, enemyHexIds):
        #print str(self.hexaMap.hexamap.x_num)+":"+str(self.hexaMap.hexamap.y_num)
        #print enemyHexIds
        self.enemyVisibility = np.zeros((self.hexaMap.hexamap.x_num, self.hexaMap.hexamap.y_num))
        
        for hexId in enemyHexIds:
            for i in range(self.hexaMap.hexamap.x_num):
                for j in range(self.hexaMap.hexamap.y_num):
                    val = self.visbilityDataMgr.getValueByHexId(hexId, i,j)
                    self.enemyVisibility[i,j] += val
                    
        minVal = self.enemyVisibility.min()
        maxVal = self.enemyVisibility.max()
        ran = float(maxVal - minVal)
        for i in range(self.x_num):
            for j in range(self.y_num):
                self.enemyVisibility[i,j] = (self.enemyVisibility[i,j]-minVal)/ran
                
        
        
            
            