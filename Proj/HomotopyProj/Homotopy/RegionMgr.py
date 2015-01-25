'''
Created on Jan 25, 2015

@author: daqing_yi
'''

class RegionMgr(object):


    def __init__(self, ray1Info, ray2Info, idx, parent):
        self.idx = idx
        self.ray1Info = ray1Info
        self.ray2Info = ray2Info
        
        self.ray1Obs = self.obstacles[ray1Info[0]]
        self.ray2Obs = self.obstacles[ray2Info[0]]
        
        if self.ray1Info[1] == "A":
            self.line1 = self.ray1Obs.alpha_seg
            self.line1_self_intsecs_info = self.ray1Obs.alpha_self_intsecs_info
            self.line1_other_intsecs_info = self.ray1Obs.alpha_obs_intsecs_info
        else:
            self.line1 = self.ray1Obs.beta_seg
            self.line1_self_intsecs_info = self.ray1Obs.beta_self_intsecs_info
            self.line1_other_intsecs_info = self.ray1Obs.beta_obs_intsecs_info
        if ray2Info[1] == "A":
            self.line2 = self.ray2Obs.alpha_seg
            self.line2_self_intsecs_info = self.ray2Obs.alpha_self_intsecs_info
            self.line2_other_intsecs_info = self.ray2Obs.alpha_obs_intsecs_info
        else:
            self.line2 = self.ray2Obs.beta_seg
            self.line2_self_intsecs_info = self.ray2Obs.beta_self_intsecs
            self.line2_other_intsecs_info = self.ray2Obs.beta_obs_intsecs
            
            
            
        
            

        