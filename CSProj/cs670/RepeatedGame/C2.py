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
    
    rounds = 200.0
    agents1[2].setProbability(0.5);
    agents2[2].setProbability(0.5);
    
    scores = np.zeros((len(agents1), len(agents2)))
    scores2 = np.zeros((len(agents1), len(agents2)))
    for r in range(len(agents1)):
        for c in range(len(agents2)):
            agents1[r].reset()
            agents2[c].reset()
            agents1[r].setPayoffMatrix(U1);
            agents2[c].setPayoffMatrix(U2);
 
            tour = Tournament(agents1[r], agents2[c]);
            tour.setPlayParam(rounds);
            
            tour.play()
            scores[r][c] = agents1[r].totalScore
            scores2[r][c] = agents2[c].totalScore 
     
    winlose = np.zeros((len(agents1), len(agents2)))       
    for r in range(len(agents1)):
        for c in range(len(agents2)):
            if scores[r][c] > scores2[r][c]:
                winlose[r][c] = 1.0;
            elif scores[r][c] == scores2[r][c]:
                winlose[r][c] = 0.0;
            elif scores[r][c] < scores2[r][c]:
                winlose[r][c] = -1.0;
    
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


    sys.stdout.write("\t")
    for a in agents2:
        sys.stdout.write(a.name + "\t")
    print
    for r in range(len(agents1)):
        sys.stdout.write(agents1[r].name + "\t")
        for c in range(len(agents2)):
            if winlose[r][c] == 1.0:
                sys.stdout.write("WIN" + "\t");
            elif winlose[r][c] == 0.0:
                sys.stdout.write("FAIR" + "\t");
            elif winlose[r][c] == -1.0:
                sys.stdout.write("LOSE" + "\t");
        print np.mean(winlose[r,:]/rounds)
        print
