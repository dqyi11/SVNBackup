'''
Created on 2013-6-17

@author: Walter
'''

from BayesNet import *
from BayesNode import *

import math
import csv

if __name__ == '__main__':
    
    closePrice = []
    volume = []
    reader = csv.reader(open("payx-2.csv"))
    '''
    for Date,Open,High,Low,Close,Volume in reader:
        print Date,Open,High,Low,Close,Volume
    '''
    for Date,Open,High,Low,Close,Volume in reader:
        closePrice.append(Close)
        volume.append(float(Volume)/1000000.0)  
        print volume

    print len(closePrice)
    #print closePrice

    Net = BayesNet()
    
    mu = InvGammaNode(var="mu", alpha=3.0, beta=0.5)
    sigma = GammaNode(var="sigma", alpha=2.0, beta=2.0)
    obsVar = GammaNode(var="obsVar", alpha=1.0, beta=2.0)
    volVar = GammaNode(var="volVar", alpha=1.0, beta=2.0)
    
    def meanFunc1(lstS, mu):
        #print mu
        #print lstS
        return (1+mu)* lstS
    def varFunc1(lstS, sigma):
        return sigma * lstS
    
    def scaleFunc1(rate):
        #print math.log(rate)
        return math.log(rate)
    
    length = 5
    S = []
    P = []
    VOL = []
    
    Net.addNode(mu)
    Net.addNode(sigma)
    Net.addNode(obsVar)
    Net.addNode(volVar)
    
    initial = {}
    candVar = {}
    evidence = {}
    queryVar = []
    
    startBias = 5
    
    for t in range(length):
        
        expName = "S-" + str(t)
        clsName = "P-" + str(t)
        
        lstExpName = "S-" + str(t-1)
        
        if t == 0:
            S.append(NormalNode(var=expName, mean=[float(closePrice[startBias]), "mu"], variance=[float(closePrice[startBias]), "sigma"], meanFunc=meanFunc1, varianceFunc=varFunc1))
        else:
            S.append(NormalNode(var=expName, mean=[lstExpName, "mu"], variance=[lstExpName, "sigma"], meanFunc=meanFunc1, varianceFunc=varFunc1))
            
        Net.addNode(S[t])
        queryVar.append(expName)
        
        # use last time close price as initial
        initial[expName] = float(closePrice[t+startBias])
        candVar[expName] = 1.0
        
        P.append(NormalNode(var=clsName, mean=expName, variance="obsVar"))
        Net.addNode(P[t])
        
        evidence[clsName] = float(closePrice[t+startBias])
        
        volName = "VOL-"+str(t)
        VOL.append(NormalNode(var=volName, mean=[expName], variance="volVar", meanFunc=scaleFunc1))
        Net.addNode(VOL[t])
        
        evidence[volName] = float(volume[t+startBias])
        
    expName = "S-" + str(length)
    clsName = "P-" + str(length)
    lstExpName = "S-" + str(length-1)
    queryP = NormalNode(var=expName, mean=[lstExpName, "mu"], variance=[lstExpName, "sigma"], meanFunc=meanFunc1, varianceFunc=varFunc1)
    queryS = NormalNode(var=clsName, mean=expName, variance="obsVar")
    initial[expName] = float(closePrice[length+startBias])
    candVar[expName] = 1.0
    initial[clsName] = float(closePrice[length+startBias])
    candVar[clsName] = 0.1
    
    queryVar.append(expName)
    queryVar.append(clsName)
    
    Net.addNode(queryP)
    Net.addNode(queryS)
        
    Net.plot("stockModel-vol-"+str(length))
    
    initial["mu"] = 0.5
    candVar["mu"] = 0.2
    initial["sigma"] = 0.5
    candVar["sigma"] = 0.2
    initial["obsVar"] = 0.1
    candVar["obsVar"] = 0.05
    initial["volVar"] = 10.0
    candVar["volVar"] = 3.0
    queryVar.append("mu")
    queryVar.append("sigma")
    queryVar.append("obsVar")
    queryVar.append("volVar")
    
    sampleNum  = 5000
    sampleFrom = 0
    
    q = Query(queryVar, evidence)
    
    fileWriter = open('stockModel-'+str(length)+'.txt', 'w')
    samples = q.doGibbsSampleWithHM(Net, initial, candVar, sampleNum, sampleFrom, hFileWriter=fileWriter)
    
    q.plotSamples(samples, plotVars = ["mu", "sigma", "volVar", "P-"+str(length), "S-"+str(length)])
    
    fileWriter.close()