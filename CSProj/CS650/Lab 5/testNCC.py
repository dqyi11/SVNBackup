'''
Created on Nov 21, 2014

@author: daqing_yi
'''

from NCCMatching import *
import numpy as np

if __name__ == '__main__':
    
    A  = np.array([[1,2,3],[4,5,6],[7,8,9]])
    B1 = np.array([[1,2,3],[4,5,6],[7,8,9]])
    B2 = np.array([[1,2,3],[4,5,6],[7,8,7]])
    B3 = np.array([[3,1,2],[6,4,5],[9,7,8]])
    print(getNCC(A, [1,1], B1, [1,1],1))
    print(getNCC(B1, [1,1], A, [1,1],1))
    print(getNCC(A, [1,1], B2, [1,1],1))
    print(getNCC(B2, [1,1], A, [1,1],1))
    print(getNCC(A, [1,1], B3, [1,1],1))
    print(getNCC(B3, [1,1], A, [1,1],1))