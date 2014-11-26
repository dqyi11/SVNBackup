from PyQt4 import QtGui, QtCore

class ImageViewerConfigForm(QtGui.QDialog):
    
    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        self.initUI()
        self.hide()
    
    def initUI(self):
        
        fromIdxVal = self.parentWindow.fromIdx
        toIdxVal = self.parentWindow.toIdx
        
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
                
        self.fromIdxLbl = QtGui.QLabel('From ')
        self.fromIdxEdit = QtGui.QLineEdit()
        self.fromIdxEdit.setText(QtCore.QString.number(fromIdxVal))
        
        self.toIdxLbl = QtGui.QLabel('To ')
        self.toIdxEdit = QtGui.QLineEdit()
        self.toIdxEdit.setText(QtCore.QString.number(toIdxVal))
       
        
        self.btnOK = QtGui.QPushButton("OK")
        self.btnCancel = QtGui.QPushButton("Cancel")
        self.btnOK.clicked.connect(self.update)
        self.btnCancel.clicked.connect(self.cancel)
        
        self.grid.addWidget(self.fromIdxLbl, 1, 0)
        self.grid.addWidget(self.fromIdxEdit, 1, 1)
        self.grid.addWidget(self.toIdxLbl, 1, 2)
        self.grid.addWidget(self.toIdxEdit, 1, 3)
        self.grid.addWidget(self.btnOK, 2, 1)
        self.grid.addWidget(self.btnCancel, 2, 2)       
        
        self.setLayout(self.grid)
        #self.setGeometry(100,100,200,400)
        self.show()
        
    def update(self):
        
        fromIdxVal, fromIdxValRet = self.fromIdxEdit.displayText().toInt()
        toIdxVal, toIdxValRet = self.toIdxEdit.displayText().toInt()
           
        if fromIdxValRet == True:   
            self.parentWindow.fromIdx = fromIdxVal
        if toIdxValRet == True:
            self.parentWindow.toIdx = toIdxVal
        
        self.hide()
        
    def cancel(self):
        self.fromIdxEdit.undo()
        self.toIdxEdit.undo()
        self.hide()
        
            