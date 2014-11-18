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
        
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
                
        self.sampleNumberLbl = QtGui.QLabel('Sample Number')
        self.sampleNumberEdit = QtGui.QLineEdit()
        self.sampleNumberEdit.setText(QtCore.QString.number(sampleNumVal))
        
        self.populationNumLbl = QtGui.QLabel('Population Number')
        self.populationNumEdit = QtGui.QLineEdit()
        self.populationNumEdit.setText(QtCore.QString.number(populationNumVal))
       
        
        self.btnOK = QtGui.QPushButton("OK")
        self.btnCancel = QtGui.QPushButton("Cancel")
        self.btnOK.clicked.connect(self.update)
        self.btnCancel.clicked.connect(self.cancel)
        
        self.grid.addWidget(self.sampleNumberLbl, 1, 0)
        self.grid.addWidget(self.sampleNumberEdit, 1, 1)
        self.grid.addWidget(self.populationNumLbl, 2, 0)
        self.grid.addWidget(self.populationNumEdit, 2, 1)
        self.grid.addWidget(self.btnOK, 3, 1)
        self.grid.addWidget(self.btnCancel, 3, 2)       
        
        self.setLayout(self.grid)
        #self.setGeometry(100,100,200,400)
        self.show()
        
    def update(self):
        
        sampleNumVal = self.sampleNumberEdit.displayText().toFloat()
        populationNumVal = self.populationNumEdit.displayText().toFloat()
           
        self.parentWindow.sampleNum = sampleNumVal
        self.parentWindow.populationNum = populationNumVal
        
        self.hide()
        
    def cancel(self):
        self.sampleNumberEdit.undo()
        self.populationNumEdit.undo()
        self.hide()
        
            