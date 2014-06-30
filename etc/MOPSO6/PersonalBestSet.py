'''
Created on 2014-1-18

@author: Walter
'''

from scipy import stats;
from sklearn import gaussian_process;
import numpy as np;

class PersonalBestSet(object):
    '''
    classdocs
    '''

    def __init__(self, index, dim):
        '''
        Constructor
        '''
        self.index = index;
        self.nondomObs = [];
        self.domObs = [];
        self.dimNum = dim;
        self.kernel = None;
   
    def addObs(self, pos, nondom):
        
        tpos = np.zeros((1, self.dimNum));
        for d in range(self.dimNum):
            tpos[0,d] = pos[0,d];
        if nondom == True:
            idx = self.getDomIndex(tpos);
            if idx >= 0:
                return False;
            idx = self.getNondomIndex(tpos);
            if idx >= 0:
                return False;
            else:
                self.nondomObs.append(tpos);
                return True;
        else:
            idx = self.getNondomIndex(tpos);
            if idx >= 0:
                del self.nondomObs[idx];
                self.domObs.append(tpos);
                return True;
            else:
                idx = self.getDomIndex(tpos);
                if idx >= 0:
                    return False;
                else:
                    self.domObs.append(tpos);
                    return True;
                    
    def getNondomIndex(self, pos):
        for i in range(len(self.nondomObs)):
            nd_pos = self.nondomObs[i];

            if self.dimNum == 1:
                if nd_pos[0,0] == pos[0,0]:
                    return i;
            elif self.dimNum == 2:
                if nd_pos[0,0] == pos[0,0] and nd_pos[0,1] == pos[0,1]:
                    return i;
        return -1;
        
    def getDomIndex(self, pos):
        for i in range(len(self.domObs)):
            d_pos = self.domObs[i];

            if self.dimNum == 1:
                if d_pos[0,0] == pos[0,0]:
                    return i;
            elif self.dimNum == 2:
                if d_pos[0,0] == pos[0,0] and d_pos[0,1] == pos[0,1]:
                    return i;        
        return -1;            
            
    def learnProb(self):
        
        ndlen = len(self.nondomObs);
        dlen = len(self.domObs);
        
        if ndlen + dlen < 3:
            return;
        
        if self.kernel == None:
            self.kernel = gaussian_process.GaussianProcess(corr='cubic',theta0=1e-2, thetaL=1e-1, thetaU=1e-1);
        
        X = np.zeros((ndlen+dlen, self.dimNum));
        Y = np.zeros((ndlen+dlen, 1));
        for i in range(ndlen):
            for d in range(self.dimNum):
                X[i, d] = self.nondomObs[i][0,d];
            Y[i,0] = 0.8;
        for i in range(dlen):
            for d in range(self.dimNum):
                X[i+ndlen, d] = self.domObs[i][0,d];
            Y[i+ndlen,0] = 1e-4;    
            
        #print X    
        #print Y
        self.kernel.fit(X, Y); 
        
    def clearObs(self):
        self.nondomObs = [];
        self.domObs = [];
                
        
    def acceptSample(self, sample):
        if self.kernel == None:
            return None;
        
        if self.dimNum == 2:
            pos = np.zeros((1,2))
            pos[0,0] = sample[0,0]
            pos[0,1] = sample[0,1]
        else:
            pos = np.zeros((1,1))
            pos[0,0] = sample[0,0]
        
        #print "pos: " + str(pos);
        
        prob = self.kernel.predict(pos, eval_MSE=True)[0]
        
        #print "prob: " + str(prob);
        
        if prob > 1:
            prob = 1;
        if prob < 0:
            prob = 0;
        
        rndVal = np.random.random();
        #print str(rndVal) + " " + str(prob);
        if rndVal <= prob:
            return True;
        else:
            return False;
        