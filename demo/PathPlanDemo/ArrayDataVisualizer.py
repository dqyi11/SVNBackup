from PyQt4 import QtGui, QtCore
import numpy as np

class ArrayDataVisualizer(QtGui.QLabel):

    def __init__(self, parent):
        super(ArrayDataVisualizer, self).__init__(parent)
        self.dataArray = None
    
    def setDataArray(self, width, height, dataArray):
        self.dataArray = self.normalize(width, height, dataArray)
        img = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
        img.fill(QtGui.QColor.white())
        for i in range(width):
            for j in range(height):
                img.setPixel(i, j, self.getColor(self.dataArray[i,j]))
                
        self.setPixmap(QtGui.QPixmap(img))
        
    def normalize(self, width, height, dataArray):
        newDataArray = np.zeros((width, height))      
        
        minVal = np.minimum(np.minimum(dataArray))
        maxVal = np.maximum(np.maximum(dataArray))
        ran = maxVal - minVal
        for i in range(width):
            for j in range(height):
                newDataArray[i,j] = dataArray[i,j]/ran
        return newDataArray
            
    def getColor(self, val):
        intVal = int(255*val)
        return QtGui.QColor(intVal, intVal, intVal)
        
        
        
        
        
        
        