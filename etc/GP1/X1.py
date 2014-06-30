'''
Created on 2014-2-17

@author: Walter
'''

import numpy as np

if __name__ == '__main__':
    
    X = np.arange(0,1,0.0002)
    
    Y =  X[None] - X[:,None]
    
    print Y.shape
    
    Z = np.linalg.inv( np.matrix(Y) )
    
    print Z * Y
    #print np.exp(-Y)