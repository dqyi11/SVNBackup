from MapViewForm import *
from PyQt4 import QtGui, QtCore
import sys

if __name__ == '__main__':
    
    app = QtGui.QApplication(sys.argv)
    form = MapViewForm([200,200])
    sys.exit(app.exec_())