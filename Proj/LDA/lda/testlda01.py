'''
Created on Dec 3, 2014

@author: daqing_yi
'''

import scipy.io
from LDASampler import *

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