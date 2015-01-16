'''
Created on Jan 15, 2015

@author: daqing_yi
'''
from scipy.misc import imread

class FitnessManager(object):

    def __init__(self):
        
        self.fitnessNum = 0
        self.currentFitnessIdx = 0
        self.fitness_list = []
        
    def addFitness(self, filename):
        self.fitness_list.append(imread(filename, True))
        self.fitnessNum = len(self.fitness_list)
    
        
    def getFitnessValue(self, idx, x, y):
        return self.fitness_list[idx][y, x]
        