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

    '''
    def learnProb(self):
        #self.nondomObs = [];
        for ps in self.pbSets:
            for obs in ps.nondomObs:
                if self.dimNum == 1:
                    if (obs in self.nondomObs) == False:
                        self.nondomObs.append(obs)
                elif self.dimNum == 2:
                    find = False;
                    for p in self.nondomObs:
                        if p[0,0] == obs[0,0] and p[0,1] == obs[0,1]:
                            find = True;
                    if find == False:
                        self.nondomObs.append(obs);
                
        if self.dimNum == 2:
            if len(self.nondomObs) > 1:
                nondomX = np.zeros(len(self.nondomObs));
                nondomY = np.zeros(len(self.nondomObs));
                for i in np.arange(len(self.nondomObs)):
                    nondomX[i] = self.nondomObs[i][0,0];
                    nondomY[i] = self.nondomObs[i][0,1];
                values = np.vstack([nondomX, nondomY]);
                self.kernel = stats.gaussian_kde(values);
        elif self.dimNum == 1:
            if len(self.nondomObs) > 1:
                nondomX = np.zeros(len(self.nondomObs));
                for i in np.arange(len(self.nondomObs)):
                    nondomX[i] = self.nondomObs[i][0,0];
                values = np.vstack([nondomX]);
                self.kernel = stats.gaussian_kde(values);
    '''
                
        
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
        