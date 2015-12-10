'''
Created on 2013-6-8

@author: Walter
'''

'''
Case 1:
Three parameters to learn
'''

from BayesNet import *
from BayesNode import *

if __name__ == '__main__':
    
    Net = BayesNet()
       
    B_T = BetaNode("Burglary:T", alpha=2.0, beta=2.0)
    MA_TT = BetaNode("MaryCalls:T|Alarm:T", alpha=2.0, beta=2.0)
    ABE_TTF = BetaNode("Alarm:T|Burglary:T,Earthquake:F", alpha=2.0, beta=2.0)
    
    '''
    B = DiscreteVarNode("Burglary")          
    E = DiscreteVarNode("Earthquake")
    A = DiscreteVarNode("Alarm", 'Burglary Earthquake')
    J = DiscreteVarNode("JohnCalls", 'Alarm')
    M = DiscreteVarNode("MaryCalls", 'Alarm')
    
    
    B[True] = "Burglary:T"
    E[True] = 0.3
    A[True,True,True] = 0.95
    A[True,True,False] = "Alarm:T|Burglary:T,Earthquake:T"
    A[True,False,True] = 0.29
    A[True,False,False] = 0.2
    J[True,True] = 0.90
    J[True,False] = 0.2
    M[True,True] = "MaryCalls:T|Alarm:T"
    M[True,False] = 0.3
    
    Net.addNode(B)
    Net.addNode(E)
    Net.addNode(A)
    Net.addNode(J)
    Net.addNode(M)
    '''
    
    Net.addNode(B_T, "blue")
    Net.addNode(MA_TT, "blue")
    Net.addNode(ABE_TTF, "blue")
    
    addObservation = True
    observationData = []
    # add observation nodes
    if addObservation == True:
        for line in open('alarmSamples100.txt'):
            instance = dict([])
            exec('instance='+line)
            observationData.append(instance)
        
    #print observationData
    
    obbB = []
    obbE = []
    obbA = []
    obbM = []
    obbJ = []
    
    obsIdx = 0
    evidenceVars = dict([])
    for obs in observationData:
        obbBName = "Burglary-{}".format(obsIdx)
        obbEName = "Earthquake-{}".format(obsIdx)
        obbAName = "Alarm-{}".format(obsIdx)
        obbJName = "JohnCalls-{}".format(obsIdx)
        obbMName = "MaryCalls-{}".format(obsIdx)
        obbBnode = DiscreteVarNode(obbBName)
        obbEnode = DiscreteVarNode(obbEName)
        obbAnode = DiscreteVarNode(obbAName, obbBName+" "+obbEName)
        obbJnode = DiscreteVarNode(obbJName, obbAName)
        obbMnode = DiscreteVarNode(obbMName, obbAName)
        
        obbBnode[True] = "Burglary:T"
        obbEnode[True] = 0.3
        obbAnode[True,True,True] = 0.95
        obbAnode[True,True,False] = "Alarm:T|Burglary:T,Earthquake:F"
        obbAnode[True,False,True] = 0.29
        obbAnode[True,False,False] = 0.2
        obbJnode[True,True] = 0.90
        obbJnode[True,False] = 0.2
        obbMnode[True,True] = "MaryCalls:T|Alarm:T"
        obbMnode[True,False] = 0.3
        
        Net.addNode(obbBnode, fillColor="gray")
        Net.addNode(obbEnode, fillColor="gray")
        Net.addNode(obbAnode, fillColor="gray")
        Net.addNode(obbJnode, fillColor="gray")
        Net.addNode(obbMnode, fillColor="gray")
        obbB.append(obbBnode)
        obbE.append(obbEnode)
        obbA.append(obbAnode)
        obbM.append(obbMnode)
        obbJ.append(obbJnode)
        evidenceVars[obbBName] = obs["Burglary"]
        evidenceVars[obbEName] = obs["Earthquake"]
        evidenceVars[obbAName] = obs["Alarm"]
        evidenceVars[obbMName] = obs["MaryCalls"] 
        evidenceVars[obbJName] = obs["JohnCalls"] 
             
        obsIdx += 1
    
    Net.init()
    
    #Net.plot('AlarmModel1')
    Net.plot('AlarmModel1-1')
    
    sampleNum = 40000
    sampleFrom = 40000
    
    print evidenceVars
    # generate samples for learning
    paramVar = ["Burglary:T", "MaryCalls:T|Alarm:T", "Alarm:T|Burglary:T,Earthquake:F"]
    #netVar = ["Burglary", "Earthquake", "Alarm", "JohnCalls", "MaryCalls"]
    #queryVar = list(set(netVar).union(set(paramVar)))
    queryVar = paramVar

    '''
    initial = {"Burglary":True,"Earthquake":False,"Alarm":False,"JohnCalls":False,"MaryCalls":False,
               "Burglary:T":0.5, "MaryCalls:T|Alarm:T":0.5, "Alarm:T|Burglary:T,Earthquake:F":0.5}
    candsd = {"Burglary:T":0.1, "MaryCalls:T|Alarm:T":0.1, "Alarm:T|Burglary:T,Earthquake:F":0.1}
    '''
    initial = {"Burglary:T":0.5, "MaryCalls:T|Alarm:T":0.5, "Alarm:T|Burglary:T,Earthquake:F":0.5}
    candsd = {"Burglary:T":0.1, "MaryCalls:T|Alarm:T":0.1, "Alarm:T|Burglary:T,Earthquake:F":0.1}

    q1 = Query(queryVar,evidenceVars)
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)
    q1.plotSamples(samples)
    
    filename = 'alarm1.txt'
    fileWriter = open(filename, 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()