'''
Created on 2013-6-13

@author: Walter
'''

from BayesNet import *
from BayesNode import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    A = DiscreteVarNode("A")          
    B = DiscreteVarNode("B", "A")
    C = DiscreteVarNode("C", "B")
    
    A[True] = 0.17
    B[True, True] = 0.3
    B[True, False] = 0.4
    C[True, True] = 0.5
    C[True, False] = 0.7
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    
    Net.init()
    
    sampleNum =50000
    sampleFrom = 1000
    
    # generate samples for learning
    q1 = Query(["A"],{})
    samples = q1.doGibbsSample(Net, sampleNum, sampleFrom)
    
    q1.plotSamples(samples)
    