'''
Created on Feb 13, 2014

@author: daqing_yi
'''

from GPClassifier import *
import numpy as np
import matplotlib.pyplot as plt

import pickle

if __name__ == '__main__':
    
    def convFuncGammaExp(X, Y, param):
        # param[0] - prefactor
        # param[1] - inverse root lengthscale
        # param[2] - gamma exponential parameters
        # param[3] - self variance term
                 
        Xdim = X.shape[0]
        Ydim = Y.shape[0]
        #print Xdim
        #print Ydim
        D = np.zeros((Xdim, Ydim))

        for i in range(Xdim):
            for j in range(Ydim):
                dist = np.linalg.norm(X[i,:] -  Y[j,:])
                D[i, j] = dist**(0.5*param[2])                
        K = param[0] * np.exp( - param[1] * D )
        #K = K + param[3] * np.eye(dim)
        
        return np.matrix(K)
        
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
    
    gpc = GPClassifier()
    gpc.setOptParam(1000, 1.0e-8)
    gpc.setCovFunc(convFuncGammaExp, covParam)
    gpc.setPriorVar(covParam[3])
    
    gpc.learn(trainDataX, trainDataC)
    testDataC = gpc.predict(testDataX)
    
    
    pickle.dump(trainDataX, open('trainDataX.data', 'wb'))
    pickle.dump(trainDataC, open('trainDataC.data', 'wb'))
        
    pickle.dump(testDataX, open('testDataX.data', 'wb'))
    pickle.dump(testDataC, open('testDataC.data', 'wb'))
    
    
    