'''
Created on 2013-11-1

@author: Walter
'''

from ImitatorDynamics import *;

if __name__ == '__main__':
    
    round = 100;
    # Prisoner's dilemma
    game1 = ImitatorDynamics(4,3,2,1,0.95);
    game1.initAgents(30, 30, [0.1, 0.3, 0.4, 0.2]);
    game1.visualize();
    game1.run(round);
    game1.plotDyno(round);
    game1.visualize();
    game1.plotAvg(round);
    
    #print game.agents;
    #print len(game.agents);
    
    # Stag hunt
#     game2 = ImitatorDynamics(3,4,2,1,0.95);
#     game2.initAgents(30, 30, [0.1, 0.3, 0.4, 0.2]);
#     game2.visualize();
#     game2.run(round);
#     game2.plotDyno(round);
#     game2.visualize();
#     game2.plotAvg(round);
    
    # Battle of the sexes
#     game3 = ImitatorDynamics(3,2,1,4,0.95);
#     game3.initAgents(30, 30, [0.1, 0.3, 0.4, 0.2]);
#     #game3.visualize()
#     game3.run(round); 
#     game3.plotDyno(round);  
#     game3.visualize() 
#     game3.plotAvg(round)
    