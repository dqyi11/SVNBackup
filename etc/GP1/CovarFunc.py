'''
Created on 2014-2-17

@author: Walter
'''
import numpy as np

class CovarFunc(object):
    '''
    classdocs
    '''


    def __init__(self, params):
        '''
        Constructor
        '''
        
    def dist(self, X, Y=None):
        
        if Y==None:
            Y = X.copy()
            
        print X.shape
        
        X1 = (X**2).sum(axis=1)
        
        print X1.shape
        D1 = np.tile(X1, (len(X1), 1))
        
        D = D1 + D1.T -2*np.dot(X,Y.T)
        
        print D.shape
        
        return D