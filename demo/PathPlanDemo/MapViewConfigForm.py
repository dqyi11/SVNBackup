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
        
        self.wingmanRadius = QtGui.QLabel('Constraint')
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
        
        '''
        self.doMultiObj = QtGui.QCheckBox("Multi-objective")
        if self.parentWindow.multiObjective == True:
            self.doMultiObj.setCheckState(QtCore.Qt.Checked)
        else:
            self.doMultiObj.setCheckState(QtCore.Qt.Unchecked)
        self.doMultiObj.stateChanged.connect(self.changeDoMultiObj)
        
        self.doExhaustive = QtGui.QCheckBox("Exhaustive")
        if self.parentWindow.useExhaustive == True:
            self.doExhaustive.setCheckState(QtCore.Qt.Checked)
        else:
            self.doExhaustive.setCheckState(QtCore.Qt.Unchecked)
        self.doExhaustive.stateChanged.connect(self.changeDoExhaustive)
        '''
        
        self.btnApply = QtGui.QPushButton("Apply")
        self.btnApply.clicked.connect(self.applyHumanPath) 
        
        self.btnPlan = QtGui.QPushButton("Plan")
        self.btnPlan.clicked.connect(self.plan)   
        
        self.btnCancel = QtGui.QPushButton("Cancel")
        self.btnCancel.clicked.connect(self.cancel)
        
        self.lbInfoVec = QtGui.QLabel("Reference path")
        self.cbInfoVec = QtGui.QComboBox()
        
        self.btnGenRef = QtGui.QPushButton("Gen Reference")
        self.btnGenRef.clicked.connect(self.genRef)
        
        self.initCbInfoVec()
        self.cbInfoVec.activated[str].connect(self.onSelcbInfoVec)
        
        self.radioBtnGroup = QtGui.QButtonGroup(self.grid)
        self.radioBtnInfoMax = QtGui.QRadioButton("Info max")
        self.radioBtnRiskMin = QtGui.QRadioButton("Risk min")
        self.radioBtnInfoMax.toggled.connect(self.updateParentState)
        self.radioBtnRiskMin.toggled.connect(self.updateParentState)
        self.radioBtnGroup.addButton(self.radioBtnInfoMax)
        self.radioBtnGroup.addButton(self.radioBtnRiskMin)
        if self.parentWindow.currentPlanState == self.parentWindow.planStates[0]:
            self.radioBtnInfoMax.setChecked(True)
        elif self.parentWindow.currentPlanState == self.parentWindow.planStates[1]:
            self.radioBtnRiskMin.setChecked(True)
        
        self.grid.addWidget(self.hexSize, 0, 0)
        self.grid.addWidget(self.hexSizeEdit, 0, 1)
        self.grid.addWidget(self.useObstacleCheck, 0, 2)
        self.grid.addWidget(self.btnGenMap, 1, 2)
        
        self.grid.addWidget(self.lbInfoVec, 2, 0)
        self.grid.addWidget(self.cbInfoVec, 2, 1)
        self.grid.addWidget(self.btnGenRef, 2, 2)
        
        self.grid.addWidget(self.radioBtnInfoMax, 3,0)
        self.grid.addWidget(self.radioBtnRiskMin, 3,1)
        
        self.grid.addWidget(self.wingmanRadius, 4, 0)
        self.grid.addWidget(self.wingmanRadiusEdit, 4, 1)
        self.grid.addWidget(self.humanObsRange, 5, 0)
        self.grid.addWidget(self.humanObsRangeEdit, 5, 1)
        self.grid.addWidget(self.humanObsFactorEdit, 5, 2)
        self.grid.addWidget(self.robotObsRange, 6, 0)
        self.grid.addWidget(self.robotObsRangeEdit, 6, 1)
        self.grid.addWidget(self.robotObsFactorEdit, 6, 2)

        self.grid.addWidget(self.btnApply, 7, 1)
        self.grid.addWidget(self.btnPlan, 7, 2)
        self.grid.addWidget(self.btnCancel, 8, 2)       
        
        self.setLayout(self.grid)
        #self.setGeometry(100,100,200,400)
        self.show()
        
    def initCbInfoVec(self):
        self.cbInfoVec.clear()
        for s in self.parentWindow.referenceStates:
            self.cbInfoVec.addItem(s)
                
    def onSelcbInfoVec(self, text):
        print text
        if text==self.parentWindow.referenceStates[0]:
            self.parentWindow.currentRefState = self.parentWindow.referenceStates[0]
        elif text==self.parentWindow.referenceStates[1]:
            self.parentWindow.currentRefState = self.parentWindow.referenceStates[1]
        elif text==self.parentWindow.referenceStates[2]:
            self.parentWindow.currentRefState = self.parentWindow.referenceStates[2]
            
        if self.parentWindow.hexaMap != None:
            self.parentWindow.clearRefPath()
            self.parentWindow.update()
        
    def changeUseObstacle(self):
        if self.useObstacleCheck.checkState() == QtCore.Qt.Checked:
            self.parentWindow.considerObstacle = True
        else:
            self.parentWindow.considerObstacle = False
        self.update()
        
    def changeDoMultiObj(self):
        if self.doMultiObj.checkState() == QtCore.Qt.Checked:
            self.parentWindow.multiObjective = True
        else:
            self.parentWindow.multiObjective = False
        self.update()
        
    def changeDoExhaustive(self):
        if self.doExhaustive.checkState() == QtCore.Qt.Checked:
            self.parentWindow.useExhaustive = True
        else:
            self.parentWindow.useExhaustive = False
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
        self.parentWindow.wingmanRadius = int(self.wingmanRadiusEdit.displayText())
        self.parentWindow.humanObsR = int(self.humanObsRangeEdit.displayText())
        self.parentWindow.robotObsR = int(self.robotObsRangeEdit.displayText())
        self.parentWindow.humanDiscountFactor = float(self.humanObsFactorEdit.displayText())
        self.parentWindow.robotDiscountFactor = float(self.robotObsFactorEdit.displayText())
        self.parentWindow.applyHumanPath()
        
    def plan(self):
        self.parentWindow.wingmanRadius = int(self.wingmanRadiusEdit.displayText())
        self.parentWindow.humanObsR = int(self.humanObsRangeEdit.displayText())
        self.parentWindow.robotObsR = int(self.robotObsRangeEdit.displayText())
        self.parentWindow.humanDiscountFactor = float(self.humanObsFactorEdit.displayText())
        self.parentWindow.robotDiscountFactor = float(self.robotObsFactorEdit.displayText())
        self.parentWindow.planPath()
        
    def genRef(self):        
        self.parentWindow.genReference()
        
    def updateParentState(self):
        if self.radioBtnInfoMax.isChecked()==True:
            self.parentWindow.updateCurrentPlanState(self.parentWindow.planStates[0])
        elif self.radioBtnRiskMin.isChecked()==True:
            self.parentWindow.updateCurrentPlanState(self.parentWindow.planStates[1])
        
            