'''
Created on Nov 20, 2014

@author: daqing_yi
'''

from networkGenerator import *
from VisualizeNetwork import *
from LDASampler import *

if __name__ == '__main__':
    
    colors = ['#FF00FF','#9370DB', '#32CD32', '#20B2AA',  '#00CED1',
              '#DAA520', '#FF1493', '#FF1493', '#778899', '#FFD700']
    
    COMMUNITY_NUM = 10
    NEIGHBOR_NUM = 50
    ACTOR_NUM = 500
    ITERATION_NUM = 30
    FOLDER = "data"
    
    netGen = networkGenerator(COMMUNITY_NUM, NEIGHBOR_NUM)
    matrix, matrix_e, matrix_c = netGen.generateSocialNetwork(ACTOR_NUM)
    
    #np.savetxt('x.txt', matrix)
    #visualizeNetwork(matrix_e, matrix_c, colors)
    
    sampler = LDASampler(COMMUNITY_NUM)
    for it, phi in enumerate(sampler.run(matrix, ITERATION_NUM)):
        print "Iteration", it
        print "Likelihood", sampler.loglikelihood()
 
        if it % 5 == 0:
            for z in range(COMMUNITY_NUM):
                sampler.save_document_image("data/community%d-%d.png" % (it,z),
                                    phi[z,:].reshape(netGen.width,-1))