'''
Created on 2014-2-17

@author: Walter
'''
from CovarFunc import *

class GaussianCovarFunc(CovarFunc):
    '''
    classdocs
    '''


    def __init__(self, params):
        '''
        Constructor
        '''
        self.params = params
        
    def K(self, X):
        
        D = np.abs(self.dist(X))
        
        K = self.params[0] * np.exp( - self.params[1] * D )
        
        return K