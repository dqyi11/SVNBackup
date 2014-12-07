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
    ITERATION_NUM = 200
    
    print classicwordlist.shape
    print truelabels.shape
    print classic400.shape

    sampler = LDASampler(TOPIC_NUM, alpha=6.667)
    for it, phi in enumerate(sampler.run(classic400, ITERATION_NUM)):
        print "Iteration", it
        likelihood = sampler.loglikelihood()
        print "Likelihood " + str(likelihood)
        
    theta = sampler.theta()
    theta_i = np.argmax(theta,axis=1) + 1
    
    print theta.shape
    print theta_i.shape
    
    wrong_cnt = 0
    for i in range(theta_i.shape[0]):
        if theta_i[i] != truelabels[0, i]:
            wrong_cnt += 1
    print " wrong cnt " + str(wrong_cnt)
    
    index1 = [index for index,value in enumerate(theta_i) if value == 1]
    index2 = [index for index,value in enumerate(theta_i) if value == 2]
    index3 = [index for index,value in enumerate(theta_i) if value == 3]
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    first_list = []
    ax.scatter(theta[index1, 0], theta[index1, 1], theta[index1, 2],c='r')
    ax.scatter(theta[index2, 0], theta[index2, 1], theta[index2, 2],c='g')
    ax.scatter(theta[index3, 0], theta[index3, 1], theta[index3, 2],c='b')
    plt.show()

    
    
    

    