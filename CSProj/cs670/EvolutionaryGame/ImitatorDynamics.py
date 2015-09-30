'''
Created on Oct 31, 2013

@author: joseph
'''

import numpy as np;
from Agent import *;
from PayoffMatrixCalc import *;
import matplotlib;
import matplotlib.pyplot as plt;
import copy

'''
imitate-the-best
'''
class ImitatorDynamics(object):
    
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
        
        ptype = np.random.random(4);
        self.pctType = np.frombuffer(ptype);
        self.pctType = self.pctType / np.sum(self.pctType);
        self.payoffMatrix = ConvertPayoff(self.T, self.R, self.P, self.S, self.gamma);
        
        self.scores = [];
        self.avScore = [];
    
    def initAgents(self, rowNum, colNum, pctType=None):
        
        agentNum = rowNum * colNum;
        self.agentNum = agentNum;
        self.rowNum = rowNum;
        self.colNum = colNum;
        agents = [];
        if pctType != None:
            self.pctType = pctType;
        self.agentTypeNum = np.multiply(agentNum, self.pctType);
        
        cnt = 0;
        for i in range(4):
            for j in range(int(self.agentTypeNum[i])):
                agents.append(i);
                cnt = cnt + 1;
                
        while(cnt < self.agentNum):
            agents.append(cnt%4);
            cnt = cnt + 1;
            
        np.random.shuffle(agents);
        
        self.agentMatrix = np.zeros((rowNum, colNum), int);
        for i in range(self.rowNum):
            for j in range(self.colNum):
                self.agentMatrix[i, j] = agents[i*colNum + j];
        
        #print self.agentMatrix;

    def findBestNeighbor(self, i, j, payoffs):
        maxP = payoffs[i, j]
        bestStrategy = self.agentMatrix[i, j]
        cnt = 0;
        avScore = 0.0;
        for r in range(-1, 2):
            for c in range (-1, 2):
                # I deliberately don't check for r == 0 and c == 0 because if the agent did better
                # than its neighbors, it should keep its strategy. 
                try:
                    if payoffs[i + r, j + c] > maxP:
                        maxP = payoffs[i + r, j + c]
                        bestStrategy = self.agentMatrix[i + r, j + c]
                    avScore = avScore + payoffs[i + r, j + c];
                    cnt = cnt + 1;
                except IndexError:
                    continue
        avScore = avScore / cnt;        
        return bestStrategy, avScore
       
        
    def calcPayoff(self, i, j):
        '''
        Finds the payoff of one agents on the lattice.
        N, NE, E, SE, S, SW, W, NW
        '''
        
        payoff = 0.0
        myStrategy = self.agentMatrix[i,j];
        neighbors = [(i-1,j), (i-1, j+1), (i, j+1), (i+1, j+1), (i+1, j), (i+1, j-1), (i, j-1), (i-1, j-1)];
        if i > 0:
            payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[0]]];
            if j > 0:
                payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[7]]];
            elif j < self.rowNum - 1:
                payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[1]]];
        
        if i < self.rowNum - 1:
            payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[4]]];
            if j > 0:
                payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[5]]];
            elif j < self.rowNum - 1:
                payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[7]]];  
                
        if j > 0:
            payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[6]]];
            
        if j < self.rowNum - 1:
            payoff += self.payoffMatrix[myStrategy, self.agentMatrix[neighbors[2]]];          
        
        return payoff;

    def calcAllPayoffs(self):
        payoffs = np.zeros((self.rowNum, self.colNum))
        for i in range(self.rowNum):
            for j in range(self.colNum):
                payoffs[i, j] = self.calcPayoff(i, j)
        return payoffs
      
    def play(self):
           
        payoffs = self.calcAllPayoffs()
        score = np.zeros(self.agentNum);
        # We need to copy the agent matrix or the agents will change strategies before they have
        # all determined which of their neighbors was best.
        agentMatrixCopy = copy.deepcopy(self.agentMatrix)
        cnt = 0;
        for i in range(self.rowNum): 
            for j in range(self.colNum):     
                agentMatrixCopy[i,j], score[cnt] = self.findBestNeighbor(i, j, payoffs);
                cnt = cnt + 1;
                
        self.scores.append(score);
        self.avScore.append(np.mean(score));
        self.agentMatrix = agentMatrixCopy
        self.pctType = self.countAgentRatios();
        
    def run(self, rounds):
        
        self.ratioDyno = np.zeros((4, rounds));
        for t in range(rounds):
            self.mutate();
            self.ratioDyno[:,t] = self.pctType;
            self.play();
            

            #print self.agentMatrix;
        #print "Agent type counts: " + str(self.countAgentRatios())
        
    def mutate(self):
        #mutProbs = np.random.random(self.agentNum)
        for i in range(self.rowNum):
            for j in range(self.colNum):
                if np.random.random() < self.mutateProb:
                    self.agentMatrix[i,j] = np.random.random_integers(0, 3, 1)[0]

    
    def countAgentRatios(self):
        ''' Returns a 4-tuple with the counts of each type of agent.'''
        counts = np.array([0, 0, 0, 0])
        for i in range(self.rowNum):
            for j in range(self.colNum):
                a = self.agentMatrix[i, j]
                counts[a] += 1
        counts = counts / float(np.sum(counts))
        return counts
    
    def visualize(self):
        
        fig = plt.figure();
        ax = fig.add_subplot(111);
        for i in range(self.rowNum):
            for j in range(self.colNum):
                if self.agentMatrix[i,j] == 0:
                    ax.plot(i+0.5,j+0.5,color='r',marker='s', markersize= 10);
                elif self.agentMatrix[i,j] == 1:
                    ax.plot(i+0.5,j+0.5,color='g',marker='s', markersize= 10);
                elif self.agentMatrix[i,j] == 2:
                    ax.plot(i+0.5,j+0.5,color='b',marker='s', markersize= 10);
                elif self.agentMatrix[i,j] == 3:
                    ax.plot(i+0.5,j+0.5,color='y',marker='s', markersize= 10);
        plt.show();

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