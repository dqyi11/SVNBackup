'''
Created on May 28, 2013

@author: walter
'''

'''
Model by Justin Page
'''

from BayesNet import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    A = BernoulliNode("A")          
    B = BernoulliNode("B")
    C = BernoulliNode("C", 'A')
    D = BernoulliNode("D", 'A B')
    E = BernoulliNode("E")
    F = BernoulliNode("F", 'C D')
    G = BernoulliNode("G", 'B D E')
    
    A['T'] = 0.51
    B['T'] = 0.31
    C['T','T'] = 0.81
    C['T','F'] = 0.31
    D['T','T','T'] = 0.99
    D['T','T','F'] = 0.73
    D['T','F','T'] = 0.62
    D['T','F','F'] = 0.17
    E['T'] = 0.75
    F['T','T','T'] = 0.11
    F['T','T','F'] = 0.98
    F['T','F','T'] = 0.89
    F['T','F','F'] = 0.13
    G['T','T','T','T'] = 0.63
    G['T','T','T','F'] = 0.99
    G['T','T','F','T'] = 0.99
    G['T','T','F','F'] = 0.81
    G['T','F','T','T'] = 0.70
    G['T','F','T','F'] = 0.05
    G['T','F','F','T'] = 0.02
    G['T','F','F','F'] = 0.78
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    Net.addNode(F)
    Net.addNode(G)
    
    Net.completeFrom(True)
    
    #Net.plot('Test1')
    
    sampleSize = 800000
    sampleFrom = 0
    
    q1 = Query(["G","F"],{"A":"T","D":"T"})
    q1.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q1.printEstProb()