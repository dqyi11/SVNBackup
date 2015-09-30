'''
Created on 2013-10-31

@author: Walter
'''

from ReplicatorDynamics import *;

if __name__ == '__main__':

#    # Prisoner's dilemma
    round = 100;
#     game1 = ReplicatorDynamics(4,3,2,1,0.95);
#     game1.initAgents(900, [0.1, 0.3, 0.4, 0.2]);
#     #game1.visualize();
#     game1.run(round);
#     game1.plotDyno(round);
#     print game1.agents
    #game1.visualize();
    #game1.plotAvg(round);
    
    #print game.agents;
    #print len(game.agents);
    
    # Stag hunt
#     game2 = ReplicatorDynamics(3,4,2,1,0.95);
#     game2.initAgents(900, [0.1, 0.3, 0.4, 0.2]);
#     game2.visualize();
#     game2.run(round);
#     game2.plotDyno(round);
#     game2.visualize();
#     game2.plotAvg(round);
    
    
    # Battle of the sexes
    game3 = ReplicatorDynamics(3,2,1,4,0.95);
    game3.initAgents(900, [0.1, 0.3, 0.4, 0.2]);
    #game3.initAgents(900, [0.2, 0.4, 0.3, 0.1]);
    game3.visualize();
    game3.run(round);
    game3.plotDyno(round);
    #print game3.pctType;
    game3.plotAvg(round);
    #game3.visualize();
   
    