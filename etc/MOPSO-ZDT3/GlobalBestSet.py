'''
Created on 2014-1-18

@author: Walter
'''
from scipy import stats;
import numpy as np;

class GlobalBestSet(object):
    '''
    classdocs
    '''

    def __init__(self, dim):
        '''
        Constructor
        '''
        self.dimNum = dim;
        self.pbSets = [];
        self.nondomObs = [];
        self.kernel = None;
        
    def addPBSet(self, pbSet):
        self.pbSets.append(pbSet);
        
    def addObs(self, obs, nondom):
        if nondom == True:
            self.nondomObs.append(obs);
            
    def clearObs(self):
        self.nondomObs = []

    def sample(self):
        nondomLen = len(self.nondomObs);
        
        if nondomLen == 0:
            return None;
        
        weights = []
        for i in range(nondomLen):
            weights.append(np.random.random());
            
        total_weight = np.sum(weights);
        for w in weights:
            w = w / total_weight;
        
        sp = np.zeros((1, self.dimNum), float);
        for d in range(self.dimNum):
            for i in range(nondomLen):
                sp[0,d] += weights[i] * self.nondomObs[i][0,d];
        
        sp = sp / nondomLen;
        
        return sp;
        