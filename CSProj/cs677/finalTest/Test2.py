'''
Created on 2013-6-19

@author: Walter
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
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    
    Net.plot("Test2")
    
    sampleNum  = 10000
    sampleFrom = 40000
    
    
    q1 = Query(["D"], {"C":34.0,"E":-24.0})
    
    initial = {"A":0.0,"B":0.0,"D":-14.0}
    candsd = {"A":2.0,"B":2.0, "D":2.0}
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)    
    q1.plotSamples(samples)
    
    import numpy as np
    import matplotlib.pyplot as plt
    dataQ = []
    for s in samples:
        dataQ.append(np.random.normal(s["D"], 0.01))
        
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(np.arange(len(dataQ)), dataQ, 'r-')
    ax1.set_xlabel(r"iteration")
    ax1.set_ylabel(r"value")
    ax1.set_title(r"Q")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    n, bins, patches = ax2.hist(dataQ, 50, normed=1)
    title = "Q" + " (Mean: {})".format(np.mean(dataQ))  
    ax2.set_title(title)  
    
    plt.show()
    
    
    fileWriter = open('test2.txt', 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()
