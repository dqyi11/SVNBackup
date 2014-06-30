from MapViewer import *
import numpy as np
from PyQt4 import QtGui, QtCore
import math

def hsv2rgb(h, s, v):
    h = float(h)
    s = float(s)
    v = float(v)
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0: r, g, b = v, t, p
    elif hi == 1: r, g, b = q, v, p
    elif hi == 2: r, g, b = p, v, t
    elif hi == 3: r, g, b = p, q, v
    elif hi == 4: r, g, b = t, p, v
    elif hi == 5: r, g, b = v, p, q
    r, g, b = int(r * 255), int(g * 255), int(b * 255)
    return r, g, b
        

class MapInfo(object):

    def __init__(self, mapviewer):
        
        self.width = mapviewer.pixmap().width()
        self.height = mapviewer.pixmap().height()
        self.pointVal = np.zeros((self.width,self.height))
        self.pointAcc = np.ndarray((self.width, self.height), np.bool)
        
        self.lightThreshold = 5
        
        mapImg = mapviewer.pixmap().toImage()
        for i in range(self.width):
            for j in range(self.height):
                c = mapImg.pixel(i,j) 
                if QtGui.QColor(c).lightness() < self.lightThreshold:
                    self.pointAcc[i,j] = False
                else:
                    self.pointAcc[i,j] = True
                    
    def getColor(self, x, y):
        return self.getRGBColor(self.pointVal[x,y])
    
    def getRGBColor(self, val):
        r,g,b = hsv2rgb(val*360,1,1)
        return r,g,b
                  

        
        
        