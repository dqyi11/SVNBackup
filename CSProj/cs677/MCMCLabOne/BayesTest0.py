'''
Created on May 29, 2013

@author: walter
'''

from BayesNet import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    A = BernoulliNode("A")          
    B = BernoulliNode("B", 'A')
    C = BernoulliNode("C", 'A B D')
    D = BernoulliNode("D", 'A')
    E = BernoulliNode("E", 'B C D')
    F = BernoulliNode("F", 'A D E')
    G = BernoulliNode("G", 'B E')
    
    A['T'] = 0.4
    B['T','T'] = 0.39
    B['T','F'] = 0.27
    C['T','T','T','T'] = 0.15
    C['T','T','T','F'] = 0.23
    C['T','T','F','T'] = 0.94
    C['T','T','F','F'] = 0.41
    C['T','F','T','T'] = 0.77
    C['T','F','T','F'] = 0.55
    C['T','F','F','T'] = 0.69
    C['T','F','F','F'] = 0.29
    D['T','T'] = 0.73
    D['T','F'] = 0.02
    E['T','T','T','T'] = 0.391
    E['T','T','T','F'] = 0.463
    E['T','T','F','T'] = 0.984
    E['T','T','F','F'] = 0.206
    E['T','F','T','T'] = 0.295
    E['T','F','T','F'] = 0.308
    E['T','F','F','T'] = 0.632
    E['T','F','F','F'] = 0.529
    F['T', 'T', 'T', 'T'] = 0.732
    F['T', 'T', 'T', 'F'] = 0.423
    F['T', 'T', 'F', 'T'] = 0.512
    F['T', 'T', 'F', 'F'] = 0.416
    F['T', 'F', 'T', 'T'] = 0.275
    F['T', 'F', 'T', 'F'] = 0.177
    F['T', 'F', 'F', 'T'] = 0.869
    F['T', 'F', 'F', 'F'] = 0.378
    G['T', 'T', 'T'] = 0.891
    G['T', 'T', 'F'] = 0.194
    G['T', 'F', 'T'] = 0.012
    G['T', 'F', 'F'] = 0.391  
    
    Net.addNode(A)
    Net.addNode(B)
    Net.addNode(C)
    Net.addNode(D)
    Net.addNode(E)
    Net.addNode(F)
    Net.addNode(G)

    Net.completeFrom(True)
    
    Net.plot('Test0')
    
    #print Net.getMarkovBlanket("E")
    
    
    sampleSize = 800000
    sampleFrom = 200000
    
    q1 = Query(["C","E"],{"A":"T","B":"T","F":"F"})
    q1.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q1.printEstProb()
    
    q2 = Query(["C", "E", "G", "F"],{"A":"F","B":"T","D":"F"})
    q2.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q2.printEstProb()
    
    q3 = Query(["A", "B", "C", "D"],{"E":"T","F":"F","G":"F"})
    q3.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q3.printEstProb()
    
    q4 = Query(["A", "C", "E", "F"],{"B":"T","G":"F"})
    q4.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q4.printEstProb()
    
    q5 = Query(["G"],{"B":"T","C":"F"})
    q5.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q5.printEstProb()
    
    q6 = Query(["B", "C", "D"],{"A":"T","E":"F"})
    q6.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q6.printEstProb()

    