'''
Created on 2013-10-25

@author: Walter
'''
import sys

if __name__ == '__main__':
    
    from Agent import *;
    from Tournament import *;
    
    agents1 = [GuessTwoStepsAgent("GT"), AlwaysDefectAgent("AD"), RandomAgent("Random"), AlwaysCooperateAgent("AC"), TitForTatAgent("TFT"), TitForTwoTatsAgent("TF2T"), NeverForgiveAgent("NF"), PavlovAgent("Pavlov"), WinStayLoseShiftAgent("WSLS")]
    agents2 = [GuessTwoStepsAgent("GT"), AlwaysDefectAgent("AD"), RandomAgent("Random"), AlwaysCooperateAgent("AC"), TitForTatAgent("TFT"), TitForTwoTatsAgent("TF2T"), NeverForgiveAgent("NF"), PavlovAgent("Pavlov"), WinStayLoseShiftAgent("WSLS")]

    #agents1 = [AlwaysDefectAgent("AD"), TitForTatAgent("TFT"), TitForTwoTatsAgent("TF2T"), NeverForgiveAgent("NF")]
    #agents2 = [AlwaysDefectAgent("AD"), TitForTatAgent("TFT"), TitForTwoTatsAgent("TF2T"), NeverForgiveAgent("NF")]

    U1 = [[3, 1], [5, 2]];
    U2 = [[3, 5], [1, 2]];
    
    rounds = 100.0
    #agents1[2].setProbability(0.8);
    #agents2[2].setProbability(0.8);
    
    scores = np.zeros((len(agents1), len(agents2)))
    scores2 = np.zeros((len(agents1), len(agents2)))
    
    avScores = np.zeros((len(agents1), len(agents2)))
    avScores2 = np.zeros((len(agents1), len(agents2)))
    
    fileWriter = open('100-run-0.75.txt', 'w')
    for it in np.arange(1,1000,1):
        for r in range(len(agents1)):
            for c in range(len(agents2)):
                agents1[r].reset()
                agents2[c].reset()
                agents1[r].setPayoffMatrix(U1);
                agents2[c].setPayoffMatrix(U2);
     
                tour = Tournament(agents1[r], agents2[c]);
                tour.setPlayParam(rounds, 0.75);
                
                tour.play()
                scores[r][c] = agents1[r].totalScore
                scores2[r][c] = agents2[r].totalScore
                
                avScores[r][c] = scores[r][c] / float(agents1[r].actionCnt)
                avScores2[r][c] = scores2[r][c] / float(agents2[r].actionCnt)
                
                #scores[r][c] = (scores[r][c] * (it-1) + agents1[r].totalScore)/float(it)
                #scores2[r][c] = (scores2[r][c] * (it-1) + agents2[c].totalScore)/float(it) 
                
        
        
        fileWriter.write(str(avScores[0][0])+","+str(avScores[0][1])+","+str(avScores[0][2])+","+str(avScores[0][3])+","+str(avScores[0][4])+","+str(avScores[0][5])+","+str(avScores[0][6])+","+str(avScores[0][7])+","+str(avScores[0][8])+","+"\n")
            
    fileWriter.close()

        
    sys.stdout.write("\t")
    for a in agents2:
        sys.stdout.write(a.name + "\t")
    print
    for r in range(len(agents1)):
        sys.stdout.write(agents1[r].name + "\t")
        for c in range(len(agents2)):
            sys.stdout.write(str(scores[r][c]/rounds) + "\t")
        print np.mean(scores[r,:]/rounds)
        print

    '''
    sys.stdout.write("\t")
    for a in agents2:
        sys.stdout.write(a.name + "\t")
    print
    for r in range(len(agents1)):
        sys.stdout.write(agents1[r].name + "\t")
        for c in range(len(agents2)):
            sys.stdout.write(str(scores2[r][c]/rounds) + "\t")
        print np.mean(scores2[r,:]/rounds)
        print
    '''