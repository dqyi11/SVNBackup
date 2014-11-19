from PyQt4 import QtGui, QtCore
from MapViewer import *
from MapViewConfigForm import *
from FitnessSpaceManager import *
from MultiObjectivePathPlanner import *

class MapViewForm(QtGui.QMainWindow):
    
    def __init__(self, size):
        super(QtGui.QMainWindow, self).__init__()
        self.formSize = size
        self.mMapViewer = MapViewer(self)
        self.resize(self.formSize[0], self.formSize[1])
        
        self.sampleNum = 15
        self.populationNum = 50
        self.generationNum = 100
        
        self.fitnessMgr = FitnessSpaceManager()
        
        self.configWindow = MapViewConfigForm(self)
        self.initUI()
        self.cursorPos = None
        
        self.planner = None
        
    def initUI(self):
        
        configAction = QtGui.QAction('Config', self)
        configAction.triggered.connect(self.configParam)        
        optAction = QtGui.QAction('Optimize', self)
        optAction.triggered.connect(self.optimize)
        importAction = QtGui.QAction('Import', self)
        importAction.triggered.connect(self.importData)
        
        menubar = self.menuBar()
        configMenu = menubar.addMenu('&Config')
        configMenu.addAction(configAction)
        
        optMenu = menubar.addMenu('&Optimize')
        optMenu.addAction(optAction)
        
        importMenu = menubar.addMenu('&Import')
        importMenu.addAction(importAction)
        
        self.initContextMenu()
        self.mMapViewer.resize(self.formSize[0], self.formSize[1])
        self.setCentralWidget(self.mMapViewer)
        
        self.show()
        
    def initContextMenu(self):
        
        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)  
        self.customContextMenuRequested.connect(self.showContextMenu) 
        
        self.contextMenu = QtGui.QMenu(self)  
        self.actionAddStart = self.contextMenu.addAction('start')
        self.actionAddEnd = self.contextMenu.addAction('end')
        
        self.actionAddStart.triggered.connect(self.addStart)
        self.actionAddEnd.triggered.connect(self.addEnd)
        
    def showContextMenu(self, pos):
        self.contextMenu.move(self.pos() + pos)
        self.cursorPos = pos
        self.contextMenu.show()
        
    def keyPressEvent(self, e):        
        if e.key() == QtCore.Qt.Key_Left:
            print "letf"
            if self.fitnessMgr.currentFitnessIdx > 0:
                self.fitnessMgr.currentFitnessIdx -= 1
                self.updateMapviewer()
                self.updateTitle()
                self.update()
        elif e.key() == QtCore.Qt.Key_Right:
            print "right"   
            if self.fitnessMgr.currentFitnessIdx < self.fitnessMgr.fitnessNum-1:
                self.fitnessMgr.currentFitnessIdx += 1
                self.updateMapviewer()
                self.updateTitle()
                self.update()
        
    def addStart(self):
        self.mMapViewer.addStart(self.cursorPos)
        
    def addEnd(self):
        self.mMapViewer.addEnd(self.cursorPos)
        
    def configParam(self):
        self.configWindow.show()
        
    def optimize(self):
        if len(self.mMapViewer.startPos)==0 or len(self.mMapViewer.endPos)==0:
            msgBox = QtGui.QMessageBox()
            msgBox.setText("Start or End has not been set.")
            msgBox.exec_()
            return            
        
        self.planner = MultiObjectivePathPlanner(self.fitnessMgr, self.mMapViewer.startPos, self.mMapViewer.endPos, self.sampleNum)
        position_range = []
        for i in range(self.sampleNum):
            position_range.append([0, self.formSize[0]-1])
            position_range.append([0, self.formSize[1]-1])
        self.mMapViewer.pathList = self.planner.findSolutions(self.populationNum, self.generationNum, position_range)
        self.update()    
        
    def updateTitle(self):
        self.setWindowTitle(str(self.fitnessMgr.currentFitnessIdx+1)+"/"+str(self.fitnessMgr.fitnessNum))
        
    def updateMapviewer(self):
        pixmap = self.fitnessMgr.pixmaps[self.fitnessMgr.currentFitnessIdx]
        self.mMapViewer.setPixmap(pixmap)
          
    def importData(self):
        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file')
        if fname=="" or fname==None:
            return
        
        pixmap = QtGui.QPixmap(fname)
        pixmap = pixmap.scaled(self.formSize[0], self.formSize[1])
        print "W:" + str(pixmap.width()) + " H:" + str(pixmap.height())
        self.fitnessMgr.addFitness(pixmap)
        self.fitnessMgr.currentFitnessIdx = self.fitnessMgr.fitnessNum-1
        self.updateMapviewer()
        self.updateTitle()
        self.update()
        
