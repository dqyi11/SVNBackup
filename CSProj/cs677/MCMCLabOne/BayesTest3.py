'''
Created on May 29, 2013

@author: walter
'''

'''
Model by HIROAKI Nakano
'''

from BayesNet import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    A = BernoulliNode("TrafficJam")
    B = BernoulliNode("BusBroken")        
    C = BernoulliNode("BusArriveOnTime",'TrafficJam BusBroken')
    D = BernoulliNode("iGotSick")
    E = BernoulliNode("BeLateForClass", 'iGotSick BusArriveOnTime')
    
    A['T'] = 0.1
    B['T'] = 0.05
    C['T','T','T'] = 0.0001
    C['T','T','F'] = 0.3
    C['T','F','T'] = 0.05
    C['T','F','F'] = 0.99
    D['T'] = 0.2
    E['T','T','T'] = 0.95
    E['T','T','F'] = 0.5
    E['T','F','T'] = 0.8
    E['T','F','F'] = 0.02                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    
    Net.completeFrom(True)
    
    Net.plot('Test3')
    
    sampleSize = 800000
    sampleFrom = 0
    
    q1 = Query(["TrafficJam"],{"BeLateForClass":"T"})
    q1.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q1.printEstProb()