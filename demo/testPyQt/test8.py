'''
Created on Apr 22, 2014

@author: walter
'''

import sys
from PyQt4 import QtGui, QtCore

class Example(QtGui.QMainWindow):
    
    def __init__(self):
        super(Example, self).__init__()
        
        self.initUI()

        
    def initUI(self):      
        
        openFile = QtGui.QAction(QtGui.QIcon('open.png'), 'Open', self)
        openFile.setShortcut('Ctrl+O')
        openFile.setStatusTip('Open new File')
        openFile.triggered.connect(self.showDialog)
        
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(openFile)

        
        self.show()       
        
    def showDialog(self):

        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file', 
                '/home')
        
        pixmap = QtGui.QPixmap(fname)

        lbl = QtGui.QLabel(self)
        lbl.setPixmap(pixmap)

        
        #qsize = QSize(pixmap.width(), int(pixmap.height()))
        #self.resize(qsize)
        self.setCentralWidget(lbl)
        self.setWindowTitle('Red Rock') 
        #self.resize(QSize(1,1))
              
         
        
def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()    