from PyQt4 import QtGui, QtCore
import numpy as np
from PIL import Image

#class ArrayDataVisualizer(QtGui.QLabel):
class ArrayDataVisualizer(object):

    def __init__(self, parent):
        #super(ArrayDataVisualizer, self).__init__(parent)
        self.dataArray = None
        self.width = 0
        self.height = 0
    
    def setDataArray(self, width, height, dataArray):
        self.width = width
        self.height = height
        self.dataArray = self.normalize(width, height, dataArray)
        '''
        img = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
        img.fill(QtGui.QColor.white())
        for i in range(width):
            for j in range(height):
                img.setPixel(i, j, self.getColor(self.dataArray[i,j]))
        #img.save('check.png')
        self.setPixmap(QtGui.QPixmap(img))
        '''
        
    def normalize(self, width, height, dataArray):
        newDataArray = np.zeros((width, height))      
        
        '''
        minArray = []
        maxArray = []
        for d in dataArray:
            minArray.append(min(d))
            maxArray.append(max(d))
        minVal = min(minArray) #np.minimum(np.minimum(dataArray))
        maxVal = max(maxArray) #np.maximum(np.maximum(dataArray))
        '''
        minVal = dataArray.min()
        maxVal = dataArray.max()
        
        ran = maxVal - minVal
        #print  str(minVal) + " - " + str(maxVal) + " = " + str(ran)
        for i in range(width):
            for j in range(height):
                if ran != 0.0:
                    newDataArray[i,j] = (dataArray[i,j] - minVal)/ran
                else:
                    newDataArray[i,j] = dataArray[i,j]
                    
        #print newDataArray            
        return newDataArray
            
    def getColor(self, val):
        intVal = int(255*val)
        return QtGui.QColor(intVal, intVal, intVal)
    
    def dumpToFile(self,filename):
    
        img = Image.new('L',(self.width, self.height), "black")
        for i in range(self.width):
            for j in range(self.height):
                intVal = int(255*self.dataArray[i,j])
                img.putpixel((i,j), intVal) 
        img.save(filename)
        
        
        
        
        
        
        