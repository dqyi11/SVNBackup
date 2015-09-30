'''
Created on May 28, 2013

@author: walter
'''

'''
Model by Daniel Johnson
'''

from BayesNet import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    A = BernoulliNode("A")          
    B = BernoulliNode("B", 'A')
    C = BernoulliNode("C")
    D = BernoulliNode("D", 'B C')
    E = BernoulliNode("E", 'C')
    
    A['T'] = 0.36
    B['T','T'] = 0.70
    B['T','F'] = 0.80
    C['T'] = 0.08
    D['T','T','T'] = 0.65
    D['T','T','F'] = 0.17
    D['T','F','T'] = 0.52
    D['T','F','F'] = 0.60
    E['T','T'] = 0.20
    E['T','F'] = 0.33
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    
    Net.completeFrom(True)
    
    Net.plot('Test2')
    
    sampleSize = 800000
    sampleFrom = 0
    
    q1 = Query(["A"],{"B":"T","E":"F"})
    q1.estProbByGibbsSample(Net, sampleSize, sampleFrom, True)
    print q1.printEstProb()
    