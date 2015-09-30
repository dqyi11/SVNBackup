'''                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
Created on May 29, 2013

@author: walter
'''

'''
Model by Joseph Heydorn
'''

from BayesNet import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    A = BernoulliNode("A")          
    B = BernoulliNode("B")
    C = BernoulliNode("C")
    D = BernoulliNode("D", 'A B C')
    E = BernoulliNode("E")
    F = BernoulliNode("F", 'C E')
    
    A['T'] = 0.9
    B['T'] = 0.2
    C['T'] = 0.777
    D['T','T','T','T'] = 0.01
    D['T','T','T','F'] = 0.1
    D['T','T','F','T'] = 0.2
    D['T','T','F','F'] = 0.3
    D['T','F','T','T'] = 0.4
    D['T','F','T','F'] = 0.5
    D['T','F','F','T'] = 0.6
    D['T','F','F','F'] = 0.7
    E['T'] = 0.001
    F['T','T','T'] = 0.001
    F['T','T','F'] = 0.5
    F['T','F','T'] = 0.1
    F['T','F','F'] = 0.9
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    Net.addNode(F)
    
    Net.completeFrom(True)
    
    Net.plot('Test4')
    
    sampleSize = 800000
    sampleFrom = 0
    
    q1 = Query(["F"],{"E":"F","A":"T"})
    q1.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q1.printEstProb()
    
    