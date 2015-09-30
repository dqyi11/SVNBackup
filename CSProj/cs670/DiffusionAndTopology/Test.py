'''
Created on 2013-11-12

@author: Walter
'''

import numpy as np;

if __name__ == '__main__':
    
    A = np.matrix([[1, 2, 3], [2, 3, 4]]);
    B = np.matrix([[1,0,0],[0,1,0],[0,0,1]]);
    
    noise = np.matrix(np.random.random((2,3)));
    
    C = B * A.T + noise.T;
    
    D = C.T
    
    print D;