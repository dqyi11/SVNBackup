'''
Created on 2013-6-11

@author: Walter
'''

from BayesNet import *
from BayesNode import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    B = DiscreteVarNode("Burglary")          
    E = DiscreteVarNode("Earthquake")
    A = DiscreteVarNode("Alarm", 'Burglary Earthquake')
    J = DiscreteVarNode("JohnCalls", 'Alarm')
    M = DiscreteVarNode("MaryCalls", 'Alarm')
    
    B[True] = 0.001
    E[True] = 0.002
    A[True,True,True] = 0.95
    A[True,True,False] = 0.94
    A[True,False,True] = 0.29
    A[True,False,False] = 0.001
    J[True,True] = 0.90
    J[True,False] = 0.05
    M[True,True] = 0.70
    M[True,False] = 0.01
    
    Net.addNode(B)
    Net.addNode(E)
    Net.addNode(A)
    Net.addNode(J)
    Net.addNode(M)
    
    Net.init()
    
    Net.plot('AlarmModel')
    
    sampleNum = 200
    sampleFrom = 40000
    
    # generate samples for learning
    q1 = Query(["Burglary", "Earthquake", "Alarm", "JohnCalls", "MaryCalls"],{})
    samples = q1.doGibbsSample(Net, sampleNum, sampleFrom)
    
    filename = 'originAlarmSamples'+str(sampleNum)+'-2.txt'
    fileWriter = open(filename, 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()
    
    
    