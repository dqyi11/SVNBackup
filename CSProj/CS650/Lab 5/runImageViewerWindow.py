'''
Created on Nov 12, 2014

@author: daqing_yi
'''

from ImageViewerWindow import *
from PyQt4 import QtGui, QtCore
import sys

if __name__ == '__main__':
    
    app = QtGui.QApplication(sys.argv)
    img_viewer = ImageViewerWindow()
    sys.exit(app.exec_())
    