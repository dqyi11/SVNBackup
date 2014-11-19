from PyQt4 import QtGui, QtCore
from MapViewer import *

class MapViewConfigForm(QtGui.QWidget):
    
    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        self.initUI()
        self.hide()
    
    def initUI(self):
        
        sampleNumVal = self.parentWindow.sampleNum
        populationNumVal = self.parentWindow.populationNum
        generationNumVal = self.parentWindow.generationNum
        
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
                
        self.sampleNumberLbl = QtGui.QLabel('Sample Number')
        self.sampleNumberEdit = QtGui.QLineEdit()
        self.sampleNumberEdit.setText(QtCore.QString.number(sampleNumVal))
        
        self.populationNumLbl = QtGui.QLabel('Population Number')
        self.populationNumEdit = QtGui.QLineEdit()
        self.populationNumEdit.setText(QtCore.QString.number(populationNumVal))
        
        self.generationNumLbl = QtGui.QLabel('Generation Number')
        self.generationNumEdit = QtGui.QLineEdit()
        self.generationNumEdit.setText(QtCore.QString.number(generationNumVal))
        
        self.btnOK = QtGui.QPushButton("OK")
        self.btnCancel = QtGui.QPushButton("Cancel")
        self.btnOK.clicked.connect(self.update)
        self.btnCancel.clicked.connect(self.cancel)
        
        self.grid.addWidget(self.sampleNumberLbl, 1, 0)
        self.grid.addWidget(self.sampleNumberEdit, 1, 1)
        self.grid.addWidget(self.populationNumLbl, 2, 0)
        self.grid.addWidget(self.populationNumEdit, 2, 1)
        self.grid.addWidget(self.generationNumLbl, 3, 0)
        self.grid.addWidget(self.generationNumEdit, 3, 1)
        self.grid.addWidget(self.btnOK, 4, 1)
        self.grid.addWidget(self.btnCancel, 4, 2)       
        
        self.setLayout(self.grid)
        #self.setGeometry(100,100,200,400)
        self.show()
        
    def update(self):
        
        sampleNumVal, sampleNumRet = self.sampleNumberEdit.displayText().toInt()
        populationNumVal, populationNumRet = self.populationNumEdit.displayText().toInt()
        generationNumVal, generationNumRet = self.generationNumEdit.displayText().toInt()
        
        if sampleNumRet == True:
            self.parentWindow.sampleNum = sampleNumVal
        if populationNumRet == True:
            self.parentWindow.populationNum = populationNumVal
        if generationNumRet == True:
            self.parentWindow.generationNum = generationNumVal
        
        self.hide()
        
    def cancel(self):
        self.sampleNumberEdit.undo()
        self.populationNumEdit.undo()
        self.generationNumEdit.undo()
        self.hide()
        
            