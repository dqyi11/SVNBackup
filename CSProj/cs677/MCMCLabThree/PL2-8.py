'''
Created on 2013-6-11

@author: Walter
'''
'''
Created on 2013-6-8

@author: Walter
'''

'''
Case 8:
All parameters to learn (1000)
'''

from BayesNet import *
from BayesNode import *

if __name__ == '__main__':
    
    Net = BayesNet()
    
    B_T = BetaNode("Burglary:T", alpha=2.0, beta=2.0)
    E_T = BetaNode("Earthquake:T", alpha=2.0, beta=2.0)
    ABE_TTT = BetaNode("Alarm:T|Burglary:T,Earthquake:T", alpha=2.0, beta=2.0)
    ABE_TTF = BetaNode("Alarm:T|Burglary:T,Earthquake:F", alpha=2.0, beta=2.0)
    ABE_TFT = BetaNode("Alarm:T|Burglary:F,Earthquake:T", alpha=2.0, beta=2.0)
    ABE_TFF = BetaNode("Alarm:T|Burglary:F,Earthquake:F", alpha=2.0, beta=2.0)
    JA_TT = BetaNode("JohnCalls:T|Alarm:T", alpha=2.0, beta=2.0)
    JA_TF = BetaNode("JohnCalls:T|Alarm:F", alpha=2.0, beta=2.0)
    MA_TT = BetaNode("MaryCalls:T|Alarm:T", alpha=2.0, beta=2.0)
    MA_TF = BetaNode("MaryCalls:T|Alarm:F", alpha=2.0, beta=2.0)
        
    '''    
    B = DiscreteVarNode("Burglary")          
    E = DiscreteVarNode("Earthquake")
    A = DiscreteVarNode("Alarm", 'Burglary Earthquake')
    J = DiscreteVarNode("JohnCalls", 'Alarm')
    M = DiscreteVarNode("MaryCalls", 'Alarm')

    B[True] = "Burglary:T"
    E[True] = "Earthquake:T"
    A[True,True,True] = "Alarm:T|Burglary:T,Earthquake:T"
    A[True,True,False] = "Alarm:T|Burglary:T,Earthquake:F"
    A[True,False,True] = "Alarm:T|Burglary:F,Earthquake:T"
    A[True,False,False] = "Alarm:T|Burglary:F,Earthquake:F"
    J[True,True] = "JohnCalls:T|Alarm:T"
    J[True,False] = "JohnCalls:T|Alarm:F"
    M[True,True] = "MaryCalls:T|Alarm:T"
    M[True,False] = "MaryCalls:T|Alarm:F"
    
    Net.addNode(B)
    Net.addNode(E)
    Net.addNode(A)
    Net.addNode(J)
    Net.addNode(M)
    '''
    
    Net.addNode(B_T, color="blue")
    Net.addNode(E_T, color="blue")
    Net.addNode(ABE_TTT, color="blue")
    Net.addNode(ABE_TTF, color="blue")
    Net.addNode(ABE_TFT, color="blue")
    Net.addNode(ABE_TFF, color="blue")
    Net.addNode(MA_TT, color="blue")
    Net.addNode(MA_TF, color="blue")
    Net.addNode(JA_TT, color="blue")
    Net.addNode(JA_TF, color="blue")

    addObservation = True
    observationData = []
    # add observation nodes
    if addObservation == True:
        for line in open('alarmSamples200.txt'):
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
        obbEnode[True] = "Earthquake:T"
        obbAnode[True,True,True] = "Alarm:T|Burglary:T,Earthquake:T"
        obbAnode[True,True,False] = "Alarm:T|Burglary:T,Earthquake:F"
        obbAnode[True,False,True] = "Alarm:T|Burglary:F,Earthquake:T"
        obbAnode[True,False,False] = "Alarm:T|Burglary:F,Earthquake:F"
        obbJnode[True,True] = "JohnCalls:T|Alarm:T"
        obbJnode[True,False] = "JohnCalls:T|Alarm:F"
        obbMnode[True,True] = "MaryCalls:T|Alarm:T"
        obbMnode[True,False] = "MaryCalls:T|Alarm:F"
        
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
    
    Net.plot('AlarmModel3-1')
    
    sampleNum = 4000
    sampleFrom = 1000
    
    # generate samples for learning
    paramVar = ["Burglary:T", "Earthquake:T", "Alarm:T|Burglary:T,Earthquake:T", "Alarm:T|Burglary:T,Earthquake:F",
                "Alarm:T|Burglary:F,Earthquake:T", "Alarm:T|Burglary:F,Earthquake:F", "JohnCalls:T|Alarm:T", 
                 "JohnCalls:T|Alarm:F", "MaryCalls:T|Alarm:T",  "MaryCalls:T|Alarm:F"]
    #netVar = ["Burglary", "Earthquake", "Alarm", "JohnCalls", "MaryCalls"]
    #queryVar = list(set(netVar).union(set(paramVar)))
    queryVar = paramVar


    initial = {"Burglary:T":0.2, "Earthquake:T":0.5, "Alarm:T|Burglary:T,Earthquake:T":0.5, "Alarm:T|Burglary:T,Earthquake:F":0.5,
               "Alarm:T|Burglary:F,Earthquake:T":0.5, "Alarm:T|Burglary:F,Earthquake:F":0.5, "MaryCalls:T|Alarm:T":0.5, 
                "MaryCalls:T|Alarm:F":0.5, "JohnCalls:T|Alarm:T":0.5, "JohnCalls:T|Alarm:F":0.5}
    candsd = {"Burglary:T":0.1, "Earthquake:T":0.1, "Alarm:T|Burglary:T,Earthquake:T":0.1, "Alarm:T|Burglary:T,Earthquake:F":0.1,
               "Alarm:T|Burglary:F,Earthquake:T":0.1, "Alarm:T|Burglary:F,Earthquake:F":0.1, "MaryCalls:T|Alarm:T":0.1, 
                "MaryCalls:T|Alarm:F":0.1, "JohnCalls:T|Alarm:T":0.1, "JohnCalls:T|Alarm:F":0.1}

    q1 = Query(queryVar,evidenceVars)
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)
    q1.plotSamples(samples)
    
    filename = 'alarm8.txt'
    fileWriter = open(filename, 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()
    