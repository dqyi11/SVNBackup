'''
Created on 2013-10-22

@author: Walter
'''

import numpy as np;

class Tournament(object):
    '''
    classdocs
    '''


    def __init__(self, agent1, agent2):
        '''
        Constructor
        '''
        self.agent1 = agent1;
        self.agent2 = agent2;
        self.roundCnt = 0;
        self.maxRoundCnt = 0;
        self.probToCont = 0.0;
        
    def roundplay(self):
        
        prevActionOfAgent1 = "C";
        prevActionOfAgent2 = "C";
        
        if self.roundCnt > 0:
            prevActionOfAgent1 = self.agent1.actions[self.roundCnt-1];
            prevActionOfAgent2 = self.agent2.actions[self.roundCnt-1];
            
        actionOfAgent1 = self.agent1.action(prevActionOfAgent2);
        actionOfAgent2 = self.agent2.action(prevActionOfAgent1);
        self.agent1.actions.append(actionOfAgent1);
        self.agent2.actions.append(actionOfAgent2);
        
        #print "Round:" + str(self.roundCnt) + " {" + actionOfAgent1 + " " + actionOfAgent2 + "}";
        
        self.calcUtility();
        
        self.agent1.update();
        self.agent2.update();
        
        self.roundCnt = self.roundCnt + 1;
        
    def play(self):
        
        for t in np.arange(1, self.maxRoundCnt+1):
            self.roundplay();
            
        execPlay = True;
        while execPlay:
        
            rndVal = np.random.random();
            #print rndVal;
            #print self.probToCont;
            if  rndVal > self.probToCont:
                execPlay = False;
            
            if execPlay == True:
                self.roundplay();
        
        self.agent1.calcTotalScore();
        self.agent2.calcTotalScore();
            
        
    def setPlayParam(self, maxRoundCnt, probToCont = 0.0):
        self.maxRoundCnt = maxRoundCnt;
        self.probToCont = probToCont;
            
    def calcUtility(self):
        
        actionOfAgent1 = self.agent1.actions[self.roundCnt-1];
        actionOfAgent2 = self.agent2.actions[self.roundCnt-1];
        
        if actionOfAgent1 == "C" and actionOfAgent2 == "C":
            scoreOfAgent1 = self.agent1.payoffMatrix[0][0];
            scoreOfAgent2 = self.agent2.payoffMatrix[0][0];
            
        elif actionOfAgent1 == "C" and actionOfAgent2 == "D":
            scoreOfAgent1 = self.agent1.payoffMatrix[0][1];
            scoreOfAgent2 = self.agent2.payoffMatrix[0][1];
            
        elif actionOfAgent1 == "D" and actionOfAgent2 == "C":
            scoreOfAgent1 = self.agent1.payoffMatrix[1][0];
            scoreOfAgent2 = self.agent2.payoffMatrix[1][0];
            
        elif actionOfAgent1 == "D" and actionOfAgent2 == "D":  
            scoreOfAgent1 = self.agent1.payoffMatrix[1][1];
            scoreOfAgent2 = self.agent2.payoffMatrix[1][1]; 
            
        self.agent1.scores.append(scoreOfAgent1);
        self.agent2.scores.append(scoreOfAgent2);
        
        
        
        
        
        
        
        