'''
Created on Nov 18, 2014

@author: daqing_yi
'''
from PyQt4 import QtGui, QtCore

class FitnessSpaceManager(object):

    def __init__(self):
        
        self.fitnessNum = 0
        self.currentFitnessIdx = 0
        self.pixmaps = []
        
    def addFitness(self, pixmap):
        self.pixmaps.append(pixmap)
        self.fitnessNum = len(self.pixmaps)
    
    def deleteFitness(self, idx):
        p = self.pixmaps[idx]
        self.pixmaps.remove(p)
        self.fitnessNum = len(self.pixmaps)
        
    def getFitnessValue(self, idx, x, y):
        int_x = int(x)
        int_y = int(y)
        img = self.pixmaps[idx].toImage()
        p = img.pixel(int_x, int_y)
        return QtGui.qGray(p)
        
        
