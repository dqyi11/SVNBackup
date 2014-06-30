'''
Created on 2014-2-16

@author: Walter
'''

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import pickle

if __name__ == '__main__':
    
    trainDataX = pickle.load(open('trainDataX.data', 'rb'))
    trainDataC = pickle.load(open('trainDataC.data', 'rb'))
    testDataX = pickle.load(open('testDataX.data', 'rb'))
    testDataC = pickle.load(open('testDataC.data', 'rb'))
    
    trainX = np.array(trainDataX.T[0,:])
    trainY = np.array(trainDataX.T[1,:])
    trainZ = np.array(trainDataC.T)[0]
    
    posX = []
    posY = []
    
    negX = []
    negY = []
    
    for i in range(len(trainZ)):
        if trainZ[i] > 0.5:
            posX.append(trainX[i])
            posY.append(trainY[i])
        else:
            negX.append(trainX[i])
            negY.append(trainY[i])
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111,)
    
    #print np.array(trainDataX[:,0].T)
    #print np.array(trainDataX[:,1].T)
    #print np.array(trainDataC.T)[0]
    
    ax.plot(posX, posY, 'sr')
    ax.plot(negX, negY, 'sb')
    

    X = np.array(testDataX.T[0,:].T)
    Y = np.array(testDataX.T[1,:].T)
    Z = np.array(testDataC)
    
    X = X.reshape(60,60)
    Y = Y.reshape(60,60)
    Z = Z.reshape(60,60)
    

    #print X
    #print Y
    #print Z
    
    levels = [0.0, 0.25, 0.5, 0.75, 1.0]
    ax.contour(X, Y, Z, levels)
    
    
    plt.show()
    
    