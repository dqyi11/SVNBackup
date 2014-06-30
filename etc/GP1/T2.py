'''
Created on Feb 13, 2014

@author: daqing_yi
'''

from GPBinaryClassifier import *
from GaussianCovarFunc import *
import numpy as np
import matplotlib.pyplot as plt

import pickle

if __name__ == '__main__':
        
    trainDataSize = 20
    trainDataX = np.random.random((trainDataSize, 2))
    trainDataX = np.vstack( ( trainDataX, np.random.random((trainDataSize,2)) + 2 * np.ones((trainDataSize,2)) ) )
    trainDataX = np.vstack( ( trainDataX, np.random.random((trainDataSize,2)) - 2 * np.ones((trainDataSize,2)) ) )
    
    trainDataC = np.zeros( (trainDataSize, 1) )
    trainDataC = np.vstack( ( trainDataC, np.ones( (2 * trainDataSize, 1) ) ) )
    
    x_flat = np.arange(-3.0, 3.0, 0.1)
    y_flat = np.arange(-3.0, 3.0, 0.1)
    x,y = np.meshgrid(x_flat,y_flat)
    
    testDataX = np.vstack((x.flat, y.flat))
    testDataX = np.matrix(testDataX).T
    
    covParam = [1, 2, 2, 0.001]
    covarFunc = GaussianCovarFunc(covParam)
    
    gpc = GPBinaryClassifier()
    gpc.setOptParam(1000, 1.0e-8)
    gpc.setCovFunc(covarFunc)
    gpc.setPriorVar(covParam[3])
    
    gpc.learn(trainDataX, trainDataC)
    testDataC = gpc.predict(testDataX)
    
    
    pickle.dump(trainDataX, open('trainDataX.data', 'wb'))
    pickle.dump(trainDataC, open('trainDataC.data', 'wb'))
        
    pickle.dump(testDataX, open('testDataX.data', 'wb'))
    pickle.dump(testDataC, open('testDataC.data', 'wb'))
    
    
    