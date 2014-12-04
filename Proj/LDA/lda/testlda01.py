'''
Created on Dec 3, 2014

@author: daqing_yi
'''

import scipy.io
from LDASampler import *
import numpy as np

if __name__ == '__main__':
    
    data = scipy.io.loadmat('classic400.mat')
    
    classicwordlist = data["classicwordlist"]
    truelabels = data["truelabels"]
    classic400 = np.array(data["classic400"].todense())
    
    TOPIC_NUM = 3
    ITERATION_NUM = 100
    
    print classicwordlist.shape
    print truelabels.shape
    print classic400.shape
    
    sampler = LDASampler(TOPIC_NUM)
    for it, phi in enumerate(sampler.run(classic400, ITERATION_NUM)):
        print "Iteration", it
        print "Likelihood", sampler.loglikelihood()
        
    phi = sampler.phi()
    print phi.shape
    np.savetxt('phi.txt', phi, delimiter= ' ' ,fmt='%1.1f' )
    
    wrong_cnt = 0
    for i in range(classicwordlist.shape[0]):
        val = phi[:,i]
        maxIdx = np.argmax(val)
        if maxIdx != truelabels[0,i]:
            wrong_cnt += 1
        
    print "Wrong count " + str(wrong_cnt)
    