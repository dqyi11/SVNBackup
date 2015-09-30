'''
Created on 2013-6-10

@author: Walter
'''

'''
Case 6:
All parameters to learn with more missing data
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
    
    Net.addNode(B, color="blue")
    Net.addNode(E, color="blue")
    Net.addNode(A, color="blue")
    Net.addNode(J, color="blue")
    Net.addNode(M, color="blue")
    
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
        for line in open('alarmSamples100-missing2.txt'):
            instance = dict([])
            exec('instance='+line)
            observationData.append(instance)
    
    obbB = []
    obbE = []
    obbA = []
    obbM = []
    obbJ = []
    
    obsIdx = 0
    evidenceVars = dict([])
    missInit = dict([])
    for obs in observationData:

        obbBName = "Burglary-{}".format(obsIdx) 
        obbBnode = DiscreteVarNode(obbBName)
        obbBnode[True] = "Burglary:T"
        Net.addNode(obbBnode, fillColor="gray")
        obbB.append(obbBnode)
        if obs.has_key("Burglary"):
            evidenceVars[obbBName] = obs["Burglary"]
        else:
            missInit[obbBName] = True   
        
        obbEName = "Earthquake-{}".format(obsIdx)
        obbEnode = DiscreteVarNode(obbEName)
        obbEnode[True] = "Earthquake:T"
        Net.addNode(obbEnode, fillColor="gray")
        obbE.append(obbEnode)
        if obs.has_key("Earthquake"):
            evidenceVars[obbEName] = obs["Earthquake"]
        else:
            missInit[obbEName] = True   
            
        obbAName = "Alarm-{}".format(obsIdx)
        obbAnode = DiscreteVarNode(obbAName, obbBName+" "+obbEName)
        obbAnode[True,True,True] = "Alarm:T|Burglary:T,Earthquake:T"
        obbAnode[True,True,False] = "Alarm:T|Burglary:T,Earthquake:F"
        obbAnode[True,False,True] = "Alarm:T|Burglary:F,Earthquake:T"
        obbAnode[True,False,False] = "Alarm:T|Burglary:F,Earthquake:F"
        Net.addNode(obbAnode, fillColor="gray")
        obbA.append(obbAnode)
        if obs.has_key("Alarm"):
            evidenceVars[obbAName] = obs["Alarm"]
        else:
            missInit[obbAName] = True   

        obbJName = "JohnCalls-{}".format(obsIdx)
        obbJnode = DiscreteVarNode(obbJName, obbAName)
        obbJnode[True,True] = "JohnCalls:T|Alarm:T"
        obbJnode[True,False] = "JohnCalls:T|Alarm:F"
        Net.addNode(obbJnode, fillColor="gray")
        obbJ.append(obbJnode)
        if obs.has_key("JohnCalls"):
            evidenceVars[obbJName] = obs["JohnCalls"]
        else:
            missInit[obbJName] = True   
        
        obbMName = "MaryCalls-{}".format(obsIdx)
        obbMnode = DiscreteVarNode(obbMName, obbAName)
        obbMnode[True,True] = "MaryCalls:T|Alarm:T"
        obbMnode[True,False] = "MaryCalls:T|Alarm:F"
        Net.addNode(obbMnode, fillColor="gray")
        obbM.append(obbMnode)
        if obs.has_key("MaryCalls"):
            evidenceVars[obbMName] = obs["MaryCalls"]
        else:
            missInit[obbMName] = True   
    
        obsIdx += 1    
    
    Net.init()    
    Net.plot('AlarmModel5')
    
    sampleNum = 40000
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
                "MaryCalls:T|Alarm:F":0.5, "JohnCalls:T|Alarm:T":0.5, "JohnCalls:T|Alarm:F":0.5,
                "Burglary":True, "Earthquake":True, "Alarm":True, "JohnCalls":True, "MaryCalls":True}
    candsd = {"Burglary:T":0.1, "Earthquake:T":0.1, "Alarm:T|Burglary:T,Earthquake:T":0.1, "Alarm:T|Burglary:T,Earthquake:F":0.1,
               "Alarm:T|Burglary:F,Earthquake:T":0.1, "Alarm:T|Burglary:F,Earthquake:F":0.1, "MaryCalls:T|Alarm:T":0.1, 
                "MaryCalls:T|Alarm:F":0.1, "JohnCalls:T|Alarm:T":0.1, "JohnCalls:T|Alarm:F":0.1}
    
    newInitial=dict(initial, **missInit) 

    q1 = Query(queryVar,evidenceVars)
    samples = q1.doGibbsSampleWithHM(Net, newInitial, candsd, sampleNum, sampleFrom)
    q1.plotSamples(samples)
    
    filename = 'alarm6.txt'
    fileWriter = open(filename, 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()