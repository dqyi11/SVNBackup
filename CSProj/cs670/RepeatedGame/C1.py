'''
Created on 2013-10-22

@author: Walter
'''

if __name__ == '__main__':
    
    from Agent import *;
    from Tournament import *;
    
    U1 = [[3, 1], [5, 2]];
    U2 = [[3, 5], [1, 2]];
    agent1 = GuessTwoStepsAgent("A1");
    #agent1 = AlwaysDefectAgent("A1");
    #agent1 = TitForTatAgent("A1");
    #agent2 = AlwaysDefectAgent("A2");
    #agent2 = RandomAgent("A2");
    #agent2.setProbability(0.8);
    #agent2 = AlwaysCooperateAgent("A2");
    #agent2 = TitForTatAgent("A2");
    agent2 = TitForTwoTatsAgent("A2");
    #agent2 = NeverForgiveAgent("A2");
    #agent2 = PavlovAgent("A2");
    #agent2 = WinStayLoseShiftAgent("A2");
    
    
    agent1.setPayoffMatrix(U1);
    agent2.setPayoffMatrix(U2);
    
    tour1 = Tournament(agent1, agent2);
    tour1.setPlayParam(100);
    
    tour1.play();
    
    print agent1.totalScore;
    print agent2.totalScore;
    
    