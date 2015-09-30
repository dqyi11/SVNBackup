'''
Created on 2013-11-7

@author: Walter
'''

from ImitatorDynamics import *;

if __name__ == '__main__':
    
    round = 20;
    # Prisoner's dilemma
    game1 = ImitatorDynamics(4,3,2,1,0.95);
    game1.initAgents(30, 30, [0.1, 0.3, 0.4, 0.2]);
    #game1.visualize();
    game1.run(round);
    #game1.plotDyno(round);
    #game1.visualize();
    #game1.plotAvg(round);
    
    game2 = ImitatorDynamics(4,3,2,1,0.95);
    game2.initAgents(10, 90, [0.1, 0.3, 0.4, 0.2]);
    game2.run(round);
    
    game3 = ImitatorDynamics(4,3,2,1,0.95);
    game3.initAgents(1, 900, [0.1, 0.3, 0.4, 0.2]);
    game3.run(round);
    
    
    
    for i in range(4):
        ind = range(round);
        fig = plt.figure();
        ax = fig.add_subplot(111);
        
        ax.plot(ind, game1.ratioDyno[i,:]);
        ax.plot(ind, game2.ratioDyno[i,:]);
        ax.plot(ind, game3.ratioDyno[i,:]);
        
        #print game1.ratioDyno[1,:];
        #print game2.ratioDyno[1,:];
        ax.set_ylabel('ratio');
        ax.set_xlabel("round");
        ax.legend(["8 neighbors", "4 neighbors", "2 neighbors"]);
        if i == 0:
            ax.set_title("Always Cooperate");
        elif i == 1:
            ax.set_title("Always Defect");
        elif i == 2:
            ax.set_title("Tit For Tat");
        elif i == 3:
            ax.set_title("Not Tit For Tat");
        plt.show();    
    