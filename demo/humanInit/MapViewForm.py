from PyQt4 import QtGui, QtCore
from MapViewer import *
from MapViewConfigForm import *

class MapViewForm(QtGui.QMainWindow):
    
    def __init__(self, size):
        super(QtGui.QMainWindow, self).__init__()
        self.formSize = size
        self.mMapViewer = MapViewer(self)
        self.resize(self.formSize[0], self.formSize[1])
        
        self.configWindow = MapViewConfigForm(self)
        self.initUI()
        self.cursorPos = None
        
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
        
    def addStart(self):
        self.mMapViewer.addStart(self.cursorPos)
        
    def addEnd(self):
        self.mMapViewer.addEnd(self.cursorPos)
        
    def configParam(self):
        self.configWindow.show()
        
    def optimize(self):
        self.mMapViewer.mas.evolve(500)
        self.update()
        
    def importData(self):
        
        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file')
        pixmap = QtGui.QPixmap(fname)
        
        self.mMapViewer.setPixmap(pixmap)
        
        self.formSize[0] = pixmap.width()
        self.formSize[1] = pixmap.height()
        
        self.mMapViewer.resize(self.formSize[0], self.formSize[1])
        
        self.imageData = pixmap.toImage()
        
        self.mMapViewer.updateData()
        
        self.update()
        
    def getDataVal(x, y):
        int_x = int(x)
        int_y = int(y)
        
        p = self.imageData.pixel(int_x, int_y)
        return QtGui.qGray(p)