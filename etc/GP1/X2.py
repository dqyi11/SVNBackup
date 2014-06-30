'''
Created on 2014-2-17

@author: Walter
'''

import numpy as np

if __name__ == '__main__':
    
    X = np.arange(0,1,0.0002)
    X = np.vstack((X, np.arange(0,1,0.0002)))
    X = X.T
    
    #Y = X**2
    
    Y = (X**2).sum(axis=1)

    
    Z = np.tile(Y, (len(Y),1))
    
    
    print X.shape
    print Y.shape
    print Z.shape
    
    print Z
    print Z.T
    #print np.exp(-Y)