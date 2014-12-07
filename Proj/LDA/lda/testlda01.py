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

    sampler = LDASampler(TOPIC_NUM, alpha=6.667)
    for it, phi in enumerate(sampler.run(classic400, ITERATION_NUM)):
        print "Iteration", it
        likelihood = sampler.loglikelihood()
        print "Likelihood " + str(likelihood)
        
    theta = sampler.theta()
    theta_i = np.argmax(phi,axis=0)
    
    print theta.shape
    
    index1 = [index for index,value in enumerate(theta_i) if value == 0]
    index2 = [index for index,value in enumerate(theta_i) if value == 1]
    index3 = [index for index,value in enumerate(theta_i) if value == 2]
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    first_list = []
    ax.scatter(phi[0,index1], phi[1,index1], phi[2,index1],c='r')
    ax.scatter(phi[0,index2], phi[1,index2], phi[2,index2],c='g')
    ax.scatter(phi[0,index3], phi[1,index3], phi[2,index3],c='b')
    plt.show()
    
    
    
    

    