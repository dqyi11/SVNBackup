from HexaMapWidget import *
from PyQt4 import QtGui, QtCore
import sys

if __name__ == '__main__':
    
    app = QtGui.QApplication(sys.argv)
    form = HexaMapWidget(16,16,16,"POINTY")
    
    #form.hexamap.generateTopologyGraph()
    sys.exit(app.exec_())
