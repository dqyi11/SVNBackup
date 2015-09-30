'''
Created on May 28, 2013

@author: walter
'''

if __name__ == '__main__':
    
    import numpy as np
    from BayesNet import *
    
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
    
    AllJointProb = np.zeros((2,2,2,2,2,2,2))
    totalAll = 0.0
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            for g in range(2):
                                AllJointProb[a,b,c,d,e,f,g] = np.log(E[tuple([e])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(B[tuple([b])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(A[tuple([a])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(G[tuple([g,b,d,e])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(F[tuple([f,c,d])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(D[tuple([d,a,b])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(C[tuple([c,a])])
                                AllJointProb[a,b,c,d,e,f,g] = np.exp(AllJointProb[a,b,c,d,e,f,g])
                                totalAll += AllJointProb[a,b,c,d,e,f,g]
                                
    print totalAll
    
    GFAD = np.zeros((2,2,2,2))
    AD = np.zeros((2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            for g in range(2):
                                    GFAD[g,f,a,d] += AllJointProb[a,b,c,d,e,f,g]
                                    AD[a,d] += AllJointProb[a,b,c,d,e,f,g]
    
    GF_AD = np.zeros((2,2,2,2))                  
    for g in range(2):
        for f in range(2):
            for a in range(2):
                for d in range(2):
                    GF_AD[g,f,a,d] = GFAD[g,f,a,d]/AD[a,d]
                
    print "P(G,F|A='T',D='T')="
    print GF_AD[:,:,0,0]      
    