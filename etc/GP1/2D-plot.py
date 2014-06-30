'''
Created on 2014-2-14

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
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    #print np.array(trainDataX[:,0].T)
    #print np.array(trainDataX[:,1].T)
    #print np.array(trainDataC.T)[0]
    
    ax.scatter(np.array(trainDataX[:,0].T), np.array(trainDataX[:,1].T), np.array(trainDataC.T)[0])
    
    #print np.array(testDataX.T[0,:])[0]
    #print np.array(testDataX.T[1,:])[0]
    #print np.array(testDataC.T)[0].shape
    
    '''
    delta = 0.025
    x = np.arange(-3.0, 3.0, delta)
    y = np.arange(-2.0, 2.0, delta)
    X, Y = np.meshgrid(x, y)
    Z1 = mlab.bivariate_normal(X, Y, 1.0, 1.0, 0.0, 0.0)
    Z2 = mlab.bivariate_normal(X, Y, 1.5, 0.5, 1, 1)
    # difference of Gaussians
    Z = 10.0 * (Z2 - Z1)
    '''
    
    X = np.array(testDataX.T[0,:].T)
    Y = np.array(testDataX.T[1,:].T)
    Z = np.array(testDataC)
    
    #X = X.reshape(80,80)
    #Y = Y.reshape(80,80)
    #Z = Z.reshape(80,80)
    
    print X.shape
    print Y.shape
    print Z.shape
    

    
    #ax.contour(X, Y, Z)
    
    
    
    
    '''
    print X
    print Y
    print Z
    plt.contour(X, Y, Z)
    '''
    
    #ax.scatter(np.array(testDataX[:,0].T), np.array(testDataX[:,1].T) ,np.array(testDataC.T))
    #plt.contour(np.array(testDataX[:,0].T), np.array(testDataX[:,1].T) ,np.array(testDataC.T))
    
    plt.show()
    
    