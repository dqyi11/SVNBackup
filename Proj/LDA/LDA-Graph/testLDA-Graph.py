'''
Created on Nov 20, 2014

@author: daqing_yi
'''

from networkGenerator import *
from VisualizeNetwork import *

if __name__ == '__main__':
    
    COMMUNITY_NUM = 10
    NEIGHBOR_NUM = 50
    NODE_NUM = 500
    
    netGen = networkGenerator(COMMUNITY_NUM, NEIGHBOR_NUM)
    matrix, matrix_e, matrix_c = netGen.generateSocialNetwork(NODE_NUM)
    
    #np.savetxt('x.txt', matrix)
    visualizeNetwork(matrix_e, matrix_c)