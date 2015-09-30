'''
Created on Jun 2, 2013

@author: walter
'''

from BayesNet import *
from BayesNode import *

import math

if __name__ == '__main__':
    
    Net = BayesNet()
    
    def expPi(x):
        if x < 0:
            return 0.0
        else:
            return np.power(x, math.pi)
    
    A = NormalNode(var="A", mean=20.0, variance=1.0)
    E = BetaNode(var="E", alpha=1.0, beta=1.0)
    B = GammaNode(var="B",alpha=["A"], beta=7.0, alphaFunc=expPi)
    D = BetaNode(var="D", alpha="A", beta="E")
    C = BernoulliNode(var="C", prob="D")
    F = PoissonNode(var="F", rate="D")
    G = NormalNode(var="G", mean="E", variance="F")
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    Net.addNode(F)
    Net.addNode(G)
    
    #Net.plot("WackyNetwork")
    
    sampleNum  = 80000
    sampleFrom = 40000
    
    
    q1 = Query(["A", "B", "C", "D", "E", "F", "G"], {})
    
    initial = {"A":20.0,"B":1600.0,"C":True,"D":0.9,"E":0.1,"F":2.0,"G":1.0}
    candsd = {"A":4.0,"B":10.0, "D":0.4, "E":0.4, "F":2.0, "G":2.0}
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)    
    q1.plotSamples(samples)
    
    '''
    
    q2 = Query(["A", "B", "C", "D", "E", "F"], {"G":5})
    initial = {"A":20.0,"B":2000,"C":True,"D":0.9,"E":0.1,"F":2.0}
    candsd = {"A":4.0,"B":10.0, "D":0.4, "E":0.4, "F":2.0}
    samples = q2.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)    
    q2.plotSamples(samples)
    
    q3 = Query(["A", "B", "C", "E", "F", "G"], {"D":0.5})
    initial = {"A":10.0,"B":0.3,"C":True,"D":0.5,"E":0.5,"F":2, "G":10.0}
    candsd = {"A":1,"B":2, "D":0.5, "E":0.02, "F":2, "G":2}
    samples = q3.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)    
    q3.plotSamples(samples)  
    
    '''
    
    fileWriter = open('WackyNetwork3.txt', 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()
