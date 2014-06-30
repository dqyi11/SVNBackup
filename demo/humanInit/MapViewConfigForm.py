from PyQt4 import QtGui, QtCore
from MapViewer import *

class MapViewConfigForm(QtGui.QWidget):
    
    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        self.initUI()
        self.hide()
    
    def initUI(self):
        
        sampleIntervalVal = self.parentWindow.mMapViewer.sampleInterval
        toleranceVal = self.parentWindow.mMapViewer.toleranceRegionSize
        
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
                
        self.sampleInterval = QtGui.QLabel('Interval')
        self.sampleIntervalEdit = QtGui.QLineEdit()
        self.sampleIntervalEdit.setText(QtCore.QString.number(sampleIntervalVal))
        
        self.tolerance = QtGui.QLabel('Tolerance')
        self.toleranceWidthEdit = QtGui.QLineEdit()
        self.toleranceHeightEdit = QtGui.QLineEdit()
        self.toleranceWidthEdit.setText(QtCore.QString.number(toleranceVal[0]))
        self.toleranceHeightEdit.setText(QtCore.QString.number(toleranceVal[1]))
        
        
        self.btnOK = QtGui.QPushButton("OK")
        self.btnCancel = QtGui.QPushButton("Cancel")
        self.btnOK.clicked.connect(self.update)
        self.btnCancel.clicked.connect(self.cancel)
        
        self.grid.addWidget(self.sampleInterval, 1, 0)
        self.grid.addWidget(self.sampleIntervalEdit, 1, 1)
        self.grid.addWidget(self.tolerance, 2, 0)
        self.grid.addWidget(self.toleranceWidthEdit, 2, 1)
        self.grid.addWidget(self.toleranceHeightEdit, 2, 2)
        self.grid.addWidget(self.btnOK, 3, 1)
        self.grid.addWidget(self.btnCancel, 3, 2)       
        
        self.setLayout(self.grid)
        #self.setGeometry(100,100,200,400)
        self.show()
        
    def update(self):
        
        sampleIntervalVal = self.sampleIntervalEdit.displayText().toFloat()
        toleranceVal = [0,0]
        toleranceVal[0] = self.toleranceWidthEdit.displayText().toFloat()
        toleranceVal[1] = self.toleranceHeightEdit.displayText().toFloat()       
        
        self.parentWindow.mMapViewer.sampleInterval = sampleIntervalVal
        self.parentWindow.mMapViewer.toleranceRegionSize = toleranceVal
        
        self.hide()
        
    def cancel(self):
        self.sampleIntervalEdit.undo()
        self.tolerance
        self.hide()
        
            