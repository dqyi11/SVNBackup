'''
Created on Dec 18, 2014

@author: hcmi
'''

import numpy as np

class PathGenerator(object):
    
    def __init__(self, mModel, oModel, oNoise):
        self.oNoise = oNoise
        self.mModel = mModel
        self.oModel = oModel
        
    def generate(self, length_num, init_pos, sys_input):
        pos = np.zeros((length_num, 2), np.double)
        obs = np.zeros((length_num, 2), np.double)
        n_obs1 = np.zeros((length_num, 2), np.double)
        n_obs2 = np.zeros((length_num, 2), np.double)
        n_obs3 = np.zeros((length_num, 2), np.double)
        n_obs4 = np.zeros((length_num, 2), np.double)
        cX, cY = init_pos[0], init_pos[1]
    
        if self.oNoise > 0:
            obs_noise = np.random.normal(loc=0, scale=self.oNoise, size = length_num*8)
        else:
            obs_noise = np.zeros(length_num*8)
            
        for i in range(length_num):
            new_loc = self.mModel([cX, cY], [sys_input[i,0], sys_input[i,1]])
            nX, nY = new_loc[0], new_loc[1]
            pos[i,0], pos[i,1] = nX, nY
            new_obs = self.oModel([pos[i,0], pos[i,1]])
            oA, oB = new_obs[0], new_obs[1]
            obs[i,0], obs[i,1] = oA, oB
            n_obs1[i,0], n_obs1[i,1] = oA + obs_noise[i*8], oB + obs_noise[i*8+1]
            n_obs2[i,0], n_obs2[i,1] = oA + obs_noise[i*8+2], oB + obs_noise[i*8+3]
            n_obs3[i,0], n_obs3[i,1] = oA + obs_noise[i*8+4], oB + obs_noise[i*8+5]
            n_obs4[i,0], n_obs4[i,1] = oA + obs_noise[i*8+6], oB + obs_noise[i*8+7]
            
            cX, cY = pos[i,0], pos[i,1]

        return pos, n_obs1, n_obs2, n_obs3, n_obs4
    
        

        