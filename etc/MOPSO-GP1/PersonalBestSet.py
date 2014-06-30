'''
Created on 2014-1-18

@author: Walter
'''

from scipy import stats;
from GPBinaryClassifier import *
from GaussianCovarFunc import *
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
        self.threshold = 0.6;
        
    def setRefSet(self, refSet):
        
        self.candX = refSet;
        self.prob = None;
        
    def randRefSet(self, size):
        
        refSet = [];
        for d in range(self.dimNum):
            refSet.append(np.random.random(size))
        self.candX = np.array(refSet).T;
        
        #print self.candX
   
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
            same = True;
            for d in range(self.dimNum):
                if nd_pos[0,d] != pos[0,d]:
                    same = False;
            if same == True:
                return i;
        return -1;
        
    def getDomIndex(self, pos):
        for i in range(len(self.domObs)):
            d_pos = self.domObs[i];
            same = True;
            for d in range(self.dimNum):
                if d_pos[0,d] != pos[0,d]:
                    same = False;
            if same == True:
                return i;      
        return -1;     
    
    def learnProb(self):
        
        ndlen = len(self.nondomObs);
        dlen = len(self.domObs);
        
        if ndlen + dlen < 3:
            return;
        
        if self.kernel == None:
            covParam = [1, 2, 2, 0.001]
            covarFunc = GaussianCovarFunc(covParam)

            self.kernel = GPBinaryClassifier()
            self.kernel.setOptParam(1000, 1.0e-4)
            self.kernel.setCovFunc(covarFunc)
            self.kernel.setPriorVar(covParam[3])
        
        X = np.zeros((ndlen+dlen, self.dimNum));
        Y = np.zeros((ndlen+dlen, 1));
        for i in range(ndlen):
            for d in range(self.dimNum):
                X[i, d] = self.nondomObs[i][0,d];
            Y[i,0] = 1.0;
        for i in range(dlen):
            for d in range(self.dimNum):
                X[i+ndlen, d] = self.domObs[i][0,d];
            Y[i+ndlen,0] = 0.0;    
            
        self.kernel.learn(X, Y)
        self.prob = self.kernel.predict(self.candX)
        
    def clearObs(self):
        self.nondomObs = [];
        self.domObs = [];
          
        
    def sampleFrom(self):
        if self.kernel == None:
            return None;
        
        posValIdx = []
        for i in range(len(self.prob)):
            if self.prob[i] > self.threshold:
                posValIdx.append(i)
                
        idx = np.random.random() * len(posValIdx)
        
        newSample = np.zeros((1,self.dimNum))
        for d in range(self.dimNum):
            newSample[0,d] = self.candX[idx,d]
        
        return newSample
        

        
        