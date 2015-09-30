'''
Created on 2013-6-5

@author: Walter
'''

from BayesNet import *
from BayesNode import *

import math

if __name__ == '__main__':
    
    Net = BayesNet()
    
    def sinSquareFunc(x):
        return np.sin(x)**2
    
    def multiplyFunc(x, y):
        return x * y
    
    def squareFunc(x):
        return x**2
    
    def addOneFunc(x):
        return x+1
    
    A1 = NormalNode(var="A1", mean=10.0, variance=2.0)
    E1 = GammaNode(var="E1", alpha=["A1"], beta=["B1"], alphaFunc=squareFunc, betaFunc=addOneFunc)
    B1 = GammaNode(var="B1",alpha="A1", beta=7.0)
    D1 = BetaNode(var="D1", alpha=["A1"], beta="E1", alphaFunc=squareFunc)
    C1 = BernoulliNode(var="C1", prob=["B1"], probFunc=sinSquareFunc)
    F1 = PoissonNode(var="F1", rate="A1")
    G1 = BetaNode(var="G1", alpha="E1", beta=["F1","D1"], betaFunc=multiplyFunc)
    
    Net.addNode(A1)
    Net.addNode(B1)
    Net.addNode(C1)
    Net.addNode(D1)
    Net.addNode(E1)
    Net.addNode(F1)
    Net.addNode(G1)
    
    #Net.plot("MadeUpNetwork")
    
    sampleNum  = 80000
    sampleFrom = 10000
    
    q1 = Query(["A1", "B1", "C1", "D1", "E1", "F1", "G1"], {})
    
    initial = {"A1":5.0,"B1":20,"C1":True,"D1":0.08,"E1":10.0,"F1":4.0,"G1":0.8}
    candsd  = {"A1":2.0,"B1":3.0,"D1":0.4, "E1":5.0, "F1":2.0, "G1":0.4}
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)    
    q1.plotSamples(samples)
    
    fileWriter = open('MadeUpNetwork.txt', 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()