'''
Created on Oct 16, 2013

@author: daqing_yi
'''

if __name__ == '__main__':
    
    import copy;
    import numpy as np
    import matplotlib
    import matplotlib.pyplot as plt
    from BayesNet import *
    from BayesNode import *
    
    def calcParam(samples):
        
        alphaVal = 0.0;
        betaVal = 0.0;
        
        cnt = 0.0;
        for s in samples:
            alphaVal = alphaVal * (cnt / (cnt + 1)) + s["hyperAlphaVar"] / (cnt+1);
            betaVal = betaVal * (cnt / (cnt + 1)) + s["hyperBetaVar"] / (cnt+1);
            
        return alphaVal, betaVal;

    
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
    
    print "current scores len: " + str(len(currentScores));
    print "max scores len: " + str(len(maxScores));
    
    dataLen = len(currentScores);
    
    Net = BayesNet();
    hyperAlphaVar = NormalNode(var="hyperAlphaVar", mean=5.0, variance=5.0);
    hyperBetaVar = NormalNode(var="hyperBetaVar", mean=5.0, variance=5.0);
    obsVars = []; 
    
    Net.addNode(hyperAlphaVar);
    Net.addNode(hyperBetaVar);
    
    dataObs = dict([]);
    
    initAlphaMean = 3.0;
    initAlphaVar = 2.0;
    initBetaMean = 3.0;
    initBetaVar = 2.0;
    
    initial = dict([]);
    candsd = dict([]);
    
    figs = [];
    
    confidenceVal = [];
    cntIdx = 0;
    step = 20;
    for t in np.arange(0, dataLen, step):
        
        startIdx = t;
        endIdx = t + step;
        if endIdx >= dataLen-1:
            endIdx = dataLen-1;
        for i in np.arange(startIdx, endIdx, 1):
            obsVarName = "obsVar" + str(i);
            obsVar = GammaNode(var=obsVarName, alpha="hyperAlphaVar", beta="hyperBetaVar");
            obsVars.append(obsVar);
            dataObs[obsVarName] = currentScores[i];
            Net.addNode(obsVar);
            
            initial[obsVarName] = currentScores[i];
            candsd[obsVarName] = 1.0;
            
        sampleNum  = 5000;
        sampleFrom = 1000;
        cntIdx = cntIdx + 1;
        
        #Net.plot(str(cntIdx))
        print len(Net.nodeList)
        
        initial["hyperAlphaVar"] = initAlphaMean;
        initial["hyperBetaVar"] = initBetaMean;
        candsd["hyperAlphaVar"] = initAlphaVar;
        candsd["hyperBetaVar"] = initBetaVar;
        
        q = Query(["hyperAlphaVar", "hyperBetaVar"], dataObs);
        samples = q.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom);
        
        initAlphaMean, initBetaMean = calcParam(samples);
                
        #print samples
        print str(initAlphaMean) + ", " + str(initBetaMean);
        
        x = np.linspace(0, 20, 200);
        from scipy.stats import *;
        rv = gamma(initAlphaMean, scale=1.0/initBetaMean);
        y = rv.pdf(x);
        
        confV = rv.cdf(maxScores[endIdx]);
        confidenceVal.append(confV);
        print confV;
        
        fig = plt.figure();
        ax1 = fig.add_subplot(111);
        ax1.plot(x, y);
        n, bins, patches = ax1.hist(sortedCurrentScores, 50, normed=1);
        
        #ax1.set_xlabel(r"p");
        #ax1.set_ylabel(r"1-p");
        #ax1.set_title("Player 1");
        
        figs.append(fig);
        
    fig = plt.figure();
    ax = fig.add_subplot(111);
    itVal = np.arange(len(confidenceVal));
    ax.plot(itVal, confidenceVal);

    plt.show();    
    
    #print dataObs;    
        
        