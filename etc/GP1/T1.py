'''
Created on Feb 13, 2014

@author: daqing_yi
'''

from GPBinaryClassifier import *
from GaussianCovarFunc import *
import pickle
import numpy as np


if __name__ == '__main__':

        
    trainDataSize = 20
    
    trainDataX = np.random.random( (trainDataSize,1) )
    trainDataX = np.vstack( ( trainDataX, np.random.random( (trainDataSize,1) ) + 2 * np.ones( (trainDataSize,1) ) ) )
    trainDataX = np.vstack( ( trainDataX, np.random.random( (trainDataSize,1) ) - 2 * np.ones( (trainDataSize,1) ) ) )
    
    trainDataC = np.zeros( (trainDataSize,1) )
    trainDataC = np.vstack( ( trainDataC, np.ones( (2 * trainDataSize,1) ) ) )
    
    testDataX = np.arange(-3.0, 3.0, 0.02)
    testDataX = np.array( np.matrix(testDataX).T )
    
    
    #print trainDataC.shape
    #print trainDataX.shape
    #print testDataX.shape
    
    covParam = [2, 1, 1, 0.0001]
    covarFunc = GaussianCovarFunc(covParam)
    
    gpc = GPBinaryClassifier()
    gpc.setOptParam(1000, 1.0e-8)
    gpc.setPriorVar(covParam[3])
    gpc.setCovFunc(covarFunc)
    
    gpc.learn(trainDataX, trainDataC)
    testDataC = gpc.predict(testDataX)
    
    #print np.array(testDataC.T)
    
    pickle.dump(trainDataX, open('trainDataX.data', 'wb'))
    pickle.dump(trainDataC, open('trainDataC.data', 'wb'))
        
    pickle.dump(testDataX, open('testDataX.data', 'wb'))
    pickle.dump(testDataC, open('testDataC.data', 'wb'))
    
    