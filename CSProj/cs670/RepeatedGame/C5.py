'''
Created on 2013-10-26

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
    #agent1 = AlwaysCooperateAgent("A1");
    #agent2 = AlwaysDefectAgent("A2");
    agent2 = RandomAgent("A2");
    #agent2.setProbability(0.8);
    #agent2 = AlwaysCooperateAgent("A2");
    #agent2 = TitForTatAgent("A2");
    #agent2 = TitForTwoTatsAgent("A2");
    #agent2 = NeverForgiveAgent("A2");
    #agent2 = PavlovAgent("A2");
    #agent2 = WinStayLoseShiftAgent("A2");
    
    prob = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    
    
    agent1.setPayoffMatrix(U1);
    agent2.setPayoffMatrix(U2);
    
    scores1 = np.zeros((999,9),float);
    scores2 = np.zeros((999,9),float);
    
    fileWriter = open('WEASEL_RND.txt', 'w')
    for i in np.arange(0,999,1):
        for t in np.arange(0,9,1):
            agent1.reset();
            agent2.reset();
            tour1 = Tournament(agent1, agent2);
            tour1.setPlayParam(100);
            agent2.setProbability(prob[t]);
            tour1.play();
            
            scores1[i,t] = agent1.totalScore/float(100);
            scores2[i,t] = agent2.totalScore/float(100);
    
        fileWriter.write(str(scores1[i,0])+","+str(scores1[i,1])+","+str(scores1[i,2])+","+str(scores1[i,3])+","+str(scores1[i,4])+","+str(scores1[i,5])+","+str(scores1[i,6])+","+str(scores1[i,7])+","+str(scores1[i,8])+","+"\n")
            
    fileWriter.close()