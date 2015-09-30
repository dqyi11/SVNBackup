'''
Created on 2013-11-1

@author: Walter
'''

from ReplicatorDynamics import *;
import numpy as np;
import matplotlib.pyplot as plt;

if __name__ == '__main__':
    
    trialNum = 100;
    ratioData = np.zeros((4,trialNum), np.double);
    # Prisoner's dilemma
    for i in range(trialNum):
        #game1 = ReplicatorDynamics(4,3,2,1,0.95);
        game1 = ReplicatorDynamics(4,3,2,1,0.99);
        #game1 = ReplicatorDynamics(3,4,2,1,0.95);
        #game1 = ReplicatorDynamics(3,4,2,1,0.99);
        #game1 = ReplicatorDynamics(3,2,1,4,0.95);
        #game1 = ReplicatorDynamics(3,2,1,4,0.99);
        game1.initAgents(900);
        #game1.visualize();
        game1.run(100);
        #game1.visualize();
        frqCnt = game1.countAgentRatios();
        ratioData[0, i] = frqCnt[0];
        ratioData[1, i] = frqCnt[1];
        ratioData[2, i] = frqCnt[2];
        ratioData[3, i] = frqCnt[3];
        
    mean = np.zeros(4);
    var = np.zeros(4);
    
    mean[0] = np.mean(ratioData[0,:]);
    var[0] = np.var(ratioData[0,:]);
    mean[1] = np.mean(ratioData[1,:]);
    var[1] = np.var(ratioData[1,:]);
    mean[2] = np.mean(ratioData[2,:]);
    var[2] = np.var(ratioData[2,:]);
    mean[3] = np.mean(ratioData[3,:]);
    var[3] = np.var(ratioData[3,:]);
    
    print mean;
    print var;
        
    fig = plt.figure();
    ax = fig.add_subplot(111);
    #ind = range(trialNum);
    width = 0.25;
    ind = np.array([0.0, 0.5, 1.0, 1.5]);
    ax.bar(ind, mean, width, color='black',
                yerr=var,
                error_kw=dict(elinewidth=2,ecolor='red'));
    ax.set_ylabel('ratio');
    xTickMarks = ["AC", "AD", "TfT", "Not TfT"];
    ax.set_xticks(ind+width/2);
    xtickNames = ax.set_xticklabels(xTickMarks);
    plt.show();
    
        
    