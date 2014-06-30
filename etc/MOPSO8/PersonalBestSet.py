'''
Created on 2014-1-18

@author: Walter
'''

from scipy import stats;
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
        if nondom == True:
            self.nondomObs.append(pos);
        else:
            self.domObs.append(pos);      

    def getDist(self, x, y):
        if self.dimNum==1:
            return np.abs(x[0,0]-y[0,0]);
        else:
            return np.sqrt((x[0,0]-y[0,0])**2 + (x[0,1]-y[0,1])**2);  
        
    def clearObs(self):
        self.domObs = [];
        self.nondomObs = [];      
        
        
    def AcceptSample(self, sample):
        ndlen = len(self.nondomObs);
        dlen = len(self.domObs);
        
        w = []
        x = [];
        
        for i in range(dlen):
            d = self.domObs[i];
            w.append(self.getDist(d, sample));
            x.append(-1.0);
        
        for i in range(ndlen):
            nd = self.nondomObs[i];
            w.append(self.getDist(nd, sample));
            x.append(0.8);
            
        sumW = 0.0;
        sumAll = 0.0;    
        for i in range(len(w)):
            sumW += w[i];
            sumAll += w[i] * x[i];
        
        if sumW == 0.0:
            return True;
          
        prob = sumAll / sumW;
        prob = prob / 2;
        prob += 0.5;
        
        #print prob;
        
        if np.random.random() < prob:
            return True;
        else:
            return False;
        
