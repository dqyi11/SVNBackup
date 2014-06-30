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
                
        
    def sample(self):
        if self.kernel == None:
            return None;
        
        if self.dimNum == 2:
            if len(self.nondomObs) > 1:
                X = self.kernel.resample(1000);
                idx = int(1000 * np.random.random());
                pos = np.matrix(np.zeros((1, self.dimNum), np.float));
                pos[0,0] = X[0,idx]
                pos[0,1] = X[1,idx]
                return pos;
            elif len(self.nondomObs) == 1:
                return self.nondomObs[0];
            else:
                return None;
        elif self.dimNum == 1:
            if len(self.nondomObs) > 1:
                X = self.kernel.resample(1000);
                idx = int(1000 * np.random.random());
                pos = np.matrix(np.zeros((1, self.dimNum), np.float));
                pos[0, 0] = X[0,idx]
                return pos;
            elif len(self.nondomObs) == 1:
                return None;
            else:
                return None;
        