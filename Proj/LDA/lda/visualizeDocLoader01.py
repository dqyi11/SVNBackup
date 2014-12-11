'''
Created on Dec 11, 2014

@author: daqing_yi
'''

from visualizeCluster import *
import scipy.io

data_mat = scipy.io.loadmat("doc_load01.mat")

print data_mat["TYPES"].shape
print data_mat["THETA_I"].shape

data_num = data_mat["THETA_I"].shape[1]

visualizeCluster(data_mat["THETA_I"], data_mat["TYPES"], 6, data_num, "doc_load01")
    
    