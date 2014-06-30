from PyQt4 import QtGui, QtCore
from MapViewForm import *

class MapViewConfigForm(QtGui.QWidget):
    
    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        self.initUI()
        self.hide()
    
    def initUI(self):        
        hexSize = self.parentWindow.hexSize
        wingmanR = self.parentWindow.wingmanRadius
        humanObsR = self.parentWindow.humanObsR
        robotObsR = self.parentWindow.robotObsR
        humanObsFactor = self.parentWindow.humanDiscountFactor
        robotObsFactor = self.parentWindow.robotDiscountFactor
        
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
                
        self.hexSize = QtGui.QLabel('Hex Size')
        self.hexSizeEdit = QtGui.QLineEdit()
        self.hexSizeEdit.setText(QtCore.QString.number(hexSize))
        
        self.useObstacleCheck = QtGui.QCheckBox("Consider Obstacle")
        if self.parentWindow.considerObstacle == True:
            self.useObstacleCheck.setCheckState(QtCore.Qt.Checked)
        else:
            self.useObstacleCheck.setCheckState(QtCore.Qt.Unchecked)
        self.useObstacleCheck.stateChanged.connect(self.changeUseObstacle)
        
        self.btnGenMap = QtGui.QPushButton("Gen Map")
        self.btnGenMap.clicked.connect(self.genMap)
        
        self.wingmanRadius = QtGui.QLabel('Wingman')
        self.wingmanRadiusEdit = QtGui.QLineEdit()
        self.wingmanRadiusEdit.setText(QtCore.QString.number(wingmanR))
        
        self.humanObsRange = QtGui.QLabel('Human')
        self.humanObsRangeEdit = QtGui.QLineEdit()
        self.humanObsRangeEdit.setText(QtCore.QString.number(humanObsR))
        self.humanObsFactorEdit = QtGui.QLineEdit()
        self.humanObsFactorEdit.setText(QtCore.QString.number(humanObsFactor))
        
        self.robotObsRange = QtGui.QLabel('Robot')
        self.robotObsRangeEdit = QtGui.QLineEdit()
        self.robotObsRangeEdit.setText(QtCore.QString.number(robotObsR))
        self.robotObsFactorEdit = QtGui.QLineEdit()
        self.robotObsFactorEdit.setText(QtCore.QString.number(robotObsFactor))
        
        self.btnApply = QtGui.QPushButton("Apply")
        self.btnApply.clicked.connect(self.applyHumanPath) 
        
        self.btnPlan = QtGui.QPushButton("Plan")
        self.btnPlan.clicked.connect(self.plan)   
        
        self.btnCancel = QtGui.QPushButton("Cancel")
        self.btnCancel.clicked.connect(self.cancel)
        
        self.grid.addWidget(self.hexSize, 1, 0)
        self.grid.addWidget(self.hexSizeEdit, 1, 1)
        self.grid.addWidget(self.useObstacleCheck, 1, 2)
        self.grid.addWidget(self.btnGenMap, 2, 2)
        self.grid.addWidget(self.wingmanRadius, 3, 0)
        self.grid.addWidget(self.wingmanRadiusEdit, 3, 1)
        self.grid.addWidget(self.humanObsRange, 4, 0)
        self.grid.addWidget(self.humanObsRangeEdit, 4, 1)
        self.grid.addWidget(self.humanObsFactorEdit, 4, 2)
        self.grid.addWidget(self.robotObsRange, 5, 0)
        self.grid.addWidget(self.robotObsRangeEdit, 5, 1)
        self.grid.addWidget(self.robotObsFactorEdit, 5, 2)
        self.grid.addWidget(self.btnApply, 6, 1)
        self.grid.addWidget(self.btnPlan, 6, 2)
        self.grid.addWidget(self.btnCancel, 7, 2)       
        
        self.setLayout(self.grid)
        #self.setGeometry(100,100,200,400)
        self.show()
        
    def changeUseObstacle(self):
        if self.useObstacleCheck.checkState() == QtCore.Qt.Checked:
            self.parentWindow.considerObstacle = True
        else:
            self.parentWindow.considerObstacle = False
        self.update()
        
    def genMap(self):
        
        hexSize = int(self.hexSizeEdit.displayText())
        self.parentWindow.hexSize = hexSize
        self.parentWindow.generateHexaMap()      
        #self.hide()
        
    def cancel(self):
        self.hexSizeEdit.undo()
        self.hide()
    
    def applyHumanPath(self):
        self.parentWindow.applyHumanPath()
        
    def plan(self):
        self.parentWindow.wingmanRadius = int(self.wingmanRadiusEdit.displayText())
        self.parentWindow.humanObsR = int(self.humanObsRangeEdit.displayText())
        self.parentWindow.robotObsR = int(self.robotObsRangeEdit.displayText())
        self.parentWindow.humanDiscountFactor = float(self.humanObsFactorEdit.displayText())
        self.parentWindow.robotDiscountFactor = float(self.robotObsFactorEdit.displayText())
        self.parentWindow.planPath()
        
            