'''
Created on 2013-10-31

@author: Walter
'''

import numpy as np;
from Agent import *;
from PayoffMatrixCalc import *;
import matplotlib;
import matplotlib.pyplot as plt;


'''
best response against your neighbors
'''
class ReplicatorDynamics(object):

    def __init__(self, T, R, P, S, gamma, mutateProb=0.0):
        '''
        Constructor
        '''
        self.T = T;
        self.R = R;
        self.P = P;
        self.S = S;
        self.gamma = gamma;
        self.mutateProb = mutateProb
        
        self.pctType = np.random.random(4);
        self.pctType = self.pctType / np.sum(self.pctType);
        self.payoffMatrix = ConvertPayoff(self.T, self.R, self.P, self.S, self.gamma);
        self.scores = [];
        self.avScore = [];
        
        
    def initAgents(self, agentNum, pctType=None):
        
        self.agentNum = agentNum;
        self.agents = [];
        if pctType != None:
            self.pctType = pctType;
        # The number of each type of agent.
        self.agentTypeNum = np.multiply(agentNum, self.pctType);
        
        cnt = 0;
        for i in range(4):
            for j in range(int(self.agentTypeNum[i])):
                self.agents.append(i);
                cnt = cnt + 1;
                
        while(cnt < self.agentNum):
            self.agents.append(cnt%4);
            cnt = cnt + 1;
                        
    def play(self):
        
        np.random.shuffle(self.agents);
        score = np.zeros(self.agentNum);
        for i in range(self.agentNum/2):            
            A = self.agents[i];
            B = self.agents[i+self.agentNum/2]
            if self.payoffMatrix[A, B] > self.payoffMatrix[B, A]:
                self.agents.append(A);
            elif self.payoffMatrix[A, B] < self.payoffMatrix[B, A]:
                self.agents.append(B);
            score[i] = self.payoffMatrix[A, B];
            score[i+self.agentNum/2] = self.payoffMatrix[B, A];
            
        self.scores.append(score);
        self.avScore.append(np.mean(score));
        # update the ratio
        #print "Number of agents" + str(len(self.agents));
        self.pctType = self.countAgentRatios();
        self.initAgents(self.agentNum);
        
    def visualize(self):
        
        freqCnt = self.pctType; #self.countAgentRatios();
        
        fig = plt.figure();
        ax = fig.add_subplot(111);
        
        width = 0.5
        ind = np.array([0.0, 1.0, 2.0, 3.0]);
        ax.bar(ind, freqCnt, width, color='red');
        ax.set_ylabel('ratio');
        xTickMarks = ["AC", "AD", "TfT", "Not TfT"];
        ax.set_xticks(ind+width/2);
        xtickNames = ax.set_xticklabels(xTickMarks);
        #ax.axis([0, 4, 0.0, 1.0])
        plt.show();
        
        
    def run(self, rounds):
        self.ratioDyno = np.zeros((4, rounds), np.double);
        for t in range(rounds):
            self.mutate()
            self.play();
            self.ratioDyno[:,t] = self.pctType;
            #print self.agents;
        #print "Agent type counts: " + str(self.countAgentRatios())
        
    def mutate(self):
        mutProbs = np.random.random(len(self.agents))
        for i in range(len(self.agents)):
            if mutProbs[i] < self.mutateProb:
                self.agents[i] = np.random.random_integers(0, 3, 1)[0]
    
    def countAgentRatios(self):
        ''' Returns a 4-tuple with the counts of each type of agent.'''
        counts = np.array([0, 0, 0, 0])
        for a in self.agents:
            counts[a] += 1
        counts = counts / float(np.sum(counts))
        return counts
        
        
    def plotDyno(self, round):
        
        ind = range(round);
        fig = plt.figure();
        ax = fig.add_subplot(111);
        
        ax.plot(ind, self.ratioDyno[0,:]);
        ax.plot(ind, self.ratioDyno[1,:]);
        ax.plot(ind, self.ratioDyno[2,:]);
        ax.plot(ind, self.ratioDyno[3,:]);
        ax.set_ylabel('ratio');
        ax.set_xlabel("round");
        ax.legend(["AC", "AD", "TfT", "Not TfT"]);
        plt.show();
        
    def plotAvg(self, round):
        
        ind = range(round);
        fig = plt.figure();
        ax = fig.add_subplot(111);
        
        ax.plot(ind, self.avScore);
        ax.set_ylabel('average score');
        ax.set_xlabel("round");
        plt.show();       
        
        
        
        
        
        
