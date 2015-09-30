'''
Created on 2013-11-6

@author: Walter
'''

from ReplicatorDynamics import *;
from ImitatorDynamics import *;
import numpy as np;
import matplotlib.pyplot as plt;

mutProbs = [0.0, 0.01, 0.05, 0.1];

if __name__ == '__main__':
    
    round = 100;
    
    dynos = []
    
    for t in range(len(mutProbs)):
        
        #game1 = ReplicatorDynamics(3,2,1,4,0.99, mutProbs[t]);
        #game1.initAgents(900, [0.1, 0.3, 0.4, 0.2]);
        game1 = ImitatorDynamics(3,2,1,4,0.99, mutProbs[t]);
        game1.initAgents(30, 30, [0.1, 0.3, 0.4, 0.2])
        game1.run(round);
        
#         game2 = ImitatorDynamics(3,2,1,4,0.99, mutProbs[t]);
#         game2.initAgents(30, 30, [0.1, 0.3, 0.4, 0.2]);
#         game2.run(round);
        dynos.append(game1.ratioDyno);
        
    
    for i in range(4):        
        ind = range(round);
        fig = plt.figure();
        ax = fig.add_subplot(111);
        
        for t in range(len(mutProbs)):
            ax.plot(ind, dynos[t][i,:]);
            
        ax.set_ylabel('ratio');
        ax.set_xlabel("round");
        if i == 0:
            ax.set_title("Ratio of Always Cooperate");
        elif i == 1:
            ax.set_title("Ratio of Always Defect");
        elif i == 2:
            ax.set_title("Ratio of Tit For Tat");
        elif i == 3:
            ax.set_title("Ratio of Not Tit For Tat");
        ax.legend(mutProbs);
        plt.show();
        