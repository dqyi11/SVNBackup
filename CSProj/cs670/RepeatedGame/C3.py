'''
Created on 2013-10-25

@author: Walter
'''
import sys

if __name__ == '__main__':
    
    from Agent import *;
    from Tournament import *;
    import matplotlib
    #matplotlib.use("WXAgg")
    import matplotlib.pyplot as plt
    
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
    tour1.setPlayParam(100,0.9);
    
    tour1.play();
    
    scoreLen = len(agent1.scores);
    avScore1 = np.zeros(scoreLen);
    avScore2 = np.zeros(scoreLen);
    for t in np.arange(scoreLen):
        print t
        if t==0:
            avScore1[t] = agent1.scores[t];
            avScore2[t] = agent2.scores[t];
        else:
            avScore1[t] = (avScore1[t-1]*(t-1) + agent1.scores[t])/float(t);
            avScore2[t] = (avScore1[t-1]*(t-1) + agent2.scores[t])/float(t);
            
    fig = plt.figure();
    ax = fig.add_subplot(111);
    
    ax.plot(np.arange(scoreLen), avScore1);
    ax.plot(np.arange(scoreLen), avScore2);
    #ax.plot(np.arange(scoreLen), agent2.scores);
    ax.set_xlabel("X");
    ax.set_ylabel("Y");
    ax.set_title("Hi");
    
    plt.show();
    
    