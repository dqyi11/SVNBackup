'''
Created on 2013-11-6

@author: Walter
'''

from ReplicatorDynamics import *;
from ImitatorDynamics import *;
import numpy as np;
import matplotlib.pyplot as plt;

dists = [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]];

if __name__ == '__main__':
    
    round = 500;
    
        
    game1 = ReplicatorDynamics(3,2,1,4,0.99,0.01);
    game1.initAgents(900, [0.0, 0.0, 1.0, 0.0]);
    #game1 = ImitatorDynamics(3,2,1,4,0.99, 0.01);
    #game1.initAgents(30, 30, [1.0, 0.0, 0.0, 0.0]);
    game1.visualize();
    game1.run(round);
    game1.visualize();
    game1.plotDyno(round);

    
