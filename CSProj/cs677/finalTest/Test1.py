'''
Created on Jun 2, 2013

@author: walter
'''

from BayesNet import *
from BayesNode import *

import math

if __name__ == '__main__':
    
    Net = BayesNet()
    
    def plus(x,y):
        return x + y
    
    A = NormalNode(var="A", mean=0.0, variance=123.0)
    B = NormalNode(var="B", mean=0.0, variance=145.0)
    C = NormalNode(var="C", mean="A", variance=167.0)
    D = NormalNode(var="D", mean=["A","B"], variance=189.0, meanFunc=plus)
    E = NormalNode(var="E", mean="D", variance=101.0)
    Q = NormalNode(var="Q", mean="D", variance=0.01)
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    Net.addNode(Q)
    
    Net.plot("Test1")
    
    sampleNum  = 40000
    sampleFrom = 40000
    
    
    q1 = Query(["D", "Q"], {"C":34.0,"E":-24.0})
    
    initial = {"A":0.0,"B":0.0,"D":-14.0,"Q":-14.0}
    candsd = {"A":2.0,"B":2.0, "D":2.0, "Q":2.0}
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)    
    q1.plotSamples(samples)

    
    fileWriter = open('test1.txt', 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()
