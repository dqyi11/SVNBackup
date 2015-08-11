from gmm import *
import numpy as np
from __builtin__ import True

class ParamGenerator(object):

    def __init__(self, worldViz):
        
        self.worldViz = worldViz

    def generateParams(self, num):
        params = []        
        for i in range(num):
            param = Param()
            bl = self.getBooleanList(i)
            param.w = np.random.random(len(self.worldViz.world.objects))
            for n in range(len(self.worldViz.world.objects)):
                if bl[n]==False:
                    param.w[n] = - param.w[n]
            param.scale = np.random.random(len(self.worldViz.world.objects))*80 + 100           
            params.append(param)        
        return params    
    
    def getBooleanList(self, int_num):
        obj_num = len(self.worldViz.world.objects)
        bl = np.zeros(obj_num, np.bool)
        int_1 = int_num % (2**obj_num)
        for i in range(obj_num):
            if int_1%(2**i)==1:
                bl[i] = True
        return bl        