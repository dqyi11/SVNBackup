from PyQt4 import QtGui, QtCore
from MapViewer import *
import numpy as np

class MapViewPlanForm(QtGui.QDialog):
    
    def __init__(self, parent=None):
        super(QtGui.QDialog, self).__init__(parent)
        self.parentWindow = parent
        
        self.states = {'idle':0,'setStart':1,'setEnd':2}
        self.state = self.states['idle']
        
        self.initUI()
        
    def initUI(self):
        self.setGeometry(100, 100, 120, 200)
        self.setWindowTitle('Plan')
        
        self.gridboxLayout = QtGui.QVBoxLayout()
        
        self.initPtBtn = QtGui.QCheckBox('set start/end', self)
        self.initPtBtn.stateChanged.connect(self.initPt)
        self.gridboxLayout.addWidget(self.initPtBtn)
        
        
        self.startBtn = QtGui.QRadioButton('start',self)
        self.connect(self.startBtn, QtCore.SIGNAL("clicked(bool)"), self.startClick)
        self.startBtn.setEnabled(False)
        self.gridboxLayout.addWidget(self.startBtn)
        
        self.endBtn = QtGui.QRadioButton('end',self)
        self.connect(self.startBtn, QtCore.SIGNAL("clicked(bool)"), self.endClick)
        self.endBtn.setEnabled(False)
        self.gridboxLayout.addWidget(self.endBtn)
        
        self.editDecomposeWindow = QtGui.QLineEdit(self)
        self.editDecomposeWindow.setFixedSize(100,20)
        self.gridboxLayout.addWidget(self.editDecomposeWindow)
        
        self.btnDecompose = QtGui.QPushButton('Decompose', self)
        self.btnDecompose.move(5, 65)
        self.btnDecompose.clicked.connect(self.parentWindow.decomposeMap)
        self.gridboxLayout.addWidget(self.btnDecompose)
        
        self.setLayout(self.gridboxLayout)
        
    def closeEvent (self, QCloseEvent):
        self.hide()
        
    def startClick(self):
        if True==self.startBtn.isChecked():
            self.state = self.states['setStart']
    
    def endClick(self):
        if True==self.endBtn.isChecked():
            self.state = self.states['setEnd']
    
    def initPt(self, state):
        if state == QtCore.Qt.Checked:
            self.startBtn.setEnabled(True)
            self.endBtn.setEnabled(True)
        else:
            self.startBtn.setEnabled(False)
            self.endBtn.setEnabled(False)
        
        
    

class MapViewForm(QtGui.QMainWindow):
    
    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.mMapViewer = MapViewer(self)
        self.addEnvModal = False
        
        self.initUI()
        
        self.toolWindow = MapViewPlanForm(self)
        self.toolWindow.show()
             
    def initUI(self):

        mapAction = QtGui.QAction('Map', self)
        mapAction.triggered.connect(self.showOpenFileDialog)
        
        rndEnvAction = QtGui.QAction('Random', self)
        rndEnvAction.triggered.connect(self.randEnv)
        mmEnvAction = QtGui.QAction('MultiModal',self)
        mmEnvAction.triggered.connect(self.mmEnv)
        
        showPlanDiagAction = QtGui.QAction('PlanDiag', self)
        showPlanDiagAction.triggered.connect(self.showPlanDiag)
        
        menubar = self.menuBar()
        mapMenu = menubar.addMenu('&Map')
        mapMenu.addAction(mapAction)
        envMenu = menubar.addMenu('&Env')
        envMenu.addAction(rndEnvAction)
        envMenu.addAction(mmEnvAction)
        
        self.toolbar = self.addToolBar('toolbar')
        self.toolbar.addAction(mapAction)
        self.toolbar.addAction(showPlanDiagAction)
        
        self.setCentralWidget(self.mMapViewer)
        self.mMapViewer.setMouseTracking(True)
        
        self.setWindowTitle('Map View') 
        self.setGeometry(300, 300, 200, 200)
        self.show()
        
    def mmEnv(self):
        self.addEnvModal = True
        QtGui.QApplication.setOverrideCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))
        
    def randEnv(self):
        self.mMapViewer.geneRndEnv()
        self.mMapViewer.updateEnv()
        self.update()
        
    def showPlanDiag(self):
        
        self.toolWindow.show()
        
    def showOpenFileDialog(self):

        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file', '~')
        pixmap = QtGui.QPixmap(fname)
        self.mMapViewer.setPixmap(pixmap)
        self.mMapViewer.initMapInfo()

        
    def mousePressEvent(self, QMouseEvent):
        super(MapViewForm, self).mousePressEvent(QMouseEvent)
        
        if self.addEnvModal == True:
            if QMouseEvent.button()==QtCore.Qt.RightButton:
                self.addEnvModal = False
                self.mMapViewer.addingEnvModal = False
                QtGui.QApplication.restoreOverrideCursor()
                
    def decomposeMap(self):
        print "I am pressed"


        

            
        
        
        
        
