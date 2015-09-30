'''
Created on May 29, 2013

@author: walter
'''

if __name__ == '__main__':
    
    import numpy as np
    from BayesNet import *
    
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
    
    ABCDE = np.zeros((2,2,2,2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        ABCDE[a,b,c,d,e] = np.log(A[tuple([a])])
                        ABCDE[a,b,c,d,e] += np.log(B[tuple([b])])
                        ABCDE[a,b,c,d,e] += np.log(C[tuple([c,a,b])])
                        ABCDE[a,b,c,d,e] += np.log(D[tuple([d])])
                        ABCDE[a,b,c,d,e] += np.log(E[tuple([e,d,c])])
                        ABCDE[a,b,c,d,e] = np.exp(ABCDE[tuple([a,b,c,d,e])])
                        
    AE = np.zeros((2,2))
    E = np.zeros((2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        AE[a,e] += ABCDE[a,b,c,d,e]
                        E[e] += ABCDE[a,b,c,d,e]
                        
    A_E = np.zeros((2,2))
    for a in range(2):
        for e in range(2):
            A_E[a,e] = AE[a,e]/E[e]
            
    print "P(A|E='T')="
    print A_E[:,0] 