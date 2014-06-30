'''
Created on Oct 16, 2013

@author: daqing_yi
'''

if __name__ == '__main__':
    
    import copy;
    import numpy as np
    import matplotlib
    #matplotlib.use("WXAgg")
    import matplotlib.pyplot as plt
    
    idx = []
    currentScores = []
    maxScores = []
    exploredNum = []
    
    
    for line in open('data.txt'):
        
        elements = line.split(',');
        idx.append(int(elements[0]));
        currentScores.append(float(elements[1]));
        maxScores.append(float(elements[2]));
        exploredNum.append(int(elements[3]));
        
     
    sortedCurrentScores = copy.deepcopy(currentScores);   
    sortedCurrentScores.sort();
    
    print currentScores
    print sortedCurrentScores
    
    fig1 = plt.figure();
    ax1 = fig1.add_subplot(111);
    n, bins, patches = ax1.hist(sortedCurrentScores, 50, normed=1);
    
    #ax1.plot(idx, sortedCurrentScores, "r-s");
    #ax1.set_xlabel(r"p");
    #ax1.set_ylabel(r"1-p");
    #ax1.set_title("Player 1");
    
    plt.show();
    
    
     
        