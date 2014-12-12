'''
Created on Dec 11, 2014

@author: daqing_yi
'''

import scipy.io
from LDASampler import *

if __name__ == '__main__':
    
    data_mat = scipy.io.loadmat('net_load01.mat')
    
    print data_mat["DATA_MATRIX"].shape
    print data_mat["NODES"].shape
    
    TOPIC_NUM = 10
    ITERATION_NUM = 10
    
    alpha_val = 50.0/TOPIC_NUM
    beta_val = 0.1
    
    print "Start sampling ......" + "alpha: " + str(alpha_val) + " beta:" + str(beta_val)
    
    sampler = LDASampler(TOPIC_NUM, alpha=alpha_val, beta=beta_val)
    for it, phi in enumerate(sampler.run(data_mat["DATA_MATRIX"], ITERATION_NUM)):
        print "Iteration", it
        likelihood = sampler.loglikelihood()
        print "Likelihood " + str(likelihood)
        
    theta = sampler.theta()
    theta_i = np.argmax(theta,axis=1)
    
    print theta.shape
    print theta_i.shape