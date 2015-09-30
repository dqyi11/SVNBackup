'''
Created on May 29, 2013

@author: walter
'''

if __name__ == '__main__':

    import numpy as np
    from BayesNet import *
    
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
    
    AllJointProb = np.zeros((2,2,2,2,2,2))
    total = 0.0
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            AllJointProb[a,b,c,d,e,f] = np.log(A[tuple([a])])
                            AllJointProb[a,b,c,d,e,f] += np.log(B[tuple([b])])
                            AllJointProb[a,b,c,d,e,f] += np.log(C[tuple([c])])
                            AllJointProb[a,b,c,d,e,f] += np.log(D[tuple([d,a,b,c])])
                            AllJointProb[a,b,c,d,e,f] += np.log(E[tuple([e])])
                            AllJointProb[a,b,c,d,e,f] += np.log(F[tuple([f,c,e])])
                            AllJointProb[a,b,c,d,e,f] = np.exp(AllJointProb[a,b,c,d,e,f])
                            total += AllJointProb[a,b,c,d,e,f]
                            
    print total
                            
    FEA = np.zeros((2,2,2))
    EA = np.zeros((2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            FEA[f,e,a] += AllJointProb[a,b,c,d,e,f]
                            EA[e,a] += AllJointProb[a,b,c,d,e,f]
                
    F_EA = np.zeros((2,2,2))
    for f in range(2):
        for e in range(2):
            for a in range(2):
                F_EA[f,e,a] = FEA[f,e,a] / EA[e,a]
                
    print "P(F|E='F',A='T')="
    print F_EA[:,1,0]
    