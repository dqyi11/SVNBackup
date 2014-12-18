'''
Created on Dec 18, 2014

@author: hcmi
'''

import numpy as np

class PathGenerator(object):
    
    def __init__(self, mModel, oModel, mNoise, oNoise):

        self.mNoise = mNoise
        self.oNoise = oNoise
        self.mModel = mModel
        self.oModel = oModel
        
    def generate(self, length_num, init_pos, sys_input):
        pos = np.zeros((length_num, 2), np.double)
        n_pos = np.zeros((length_num, 2), np.double)
        obs = np.zeros((length_num, 2), np.double)
        n_obs = np.zeros((length_num, 2), np.double)
        cX, cY = init_pos[0], init_pos[1]
        if self.mNoise > 0:
            pos_noise = np.random.normal(loc=0, scale=self.mNoise, size = length_num*2)
        else:
            pos_noise = np.zeros(length_num*2)
        if self.oNoise > 0:
            obs_noise = np.random.normal(loc=0, scale=self.oNoise, size = length_num*2)
        else:
            obs_noise = np.zeros(length_num*2)
            
        for i in range(length_num):
            new_loc = self.mModel([cX, cY], [sys_input[i,0], sys_input[i,1]])
            nX, nY = new_loc[0], new_loc[1]
            pos[i,0], pos[i,1] = nX, nY
            n_pos[i,0], n_pos[i,1] = nX + pos_noise[i*2], nY + pos_noise[i*2+1]
            new_obs = self.oModel([n_pos[i,0], n_pos[i,1]])
            oA, oB = new_obs[0], new_obs[1]
            obs[i,0], obs[i,1] = oA, oB
            n_obs[i,0], n_obs[i,1] = oA + obs_noise[i*2], oB + obs_noise[i*2+1]
            cX, cY = n_pos[i,0], n_pos[i,1]
        
        return pos, n_pos, obs, n_obs
    
        

        