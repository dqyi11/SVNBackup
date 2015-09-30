'''
Created on May 29, 2013

@author: walter
'''

if __name__ == '__main__':

    import numpy as np
    from BayesNet import *
    
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
    
    AllJointProb = np.zeros((2,2,2,2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        AllJointProb[a,b,c,d,e] = np.log(A[tuple([a])])
                        AllJointProb[a,b,c,d,e] += np.log(B[tuple([b,a])])
                        AllJointProb[a,b,c,d,e] += np.log(C[tuple([c])])
                        AllJointProb[a,b,c,d,e] += np.log(D[tuple([d,b,c])])
                        AllJointProb[a,b,c,d,e] += np.log(E[tuple([e,c])])
                        AllJointProb[a,b,c,d,e] = np.exp(AllJointProb[a,b,c,d,e])
                        
    ABE = np.zeros((2,2,2))
    BE = np.zeros((2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        ABE[a,b,e] += AllJointProb[a,b,c,d,e]
                        BE[b,e] += AllJointProb[a,b,c,d,e]
                        
    A_BE = np.zeros((2,2,2))
    for a in range(2):
        for b in range(2):
            for e in range(2):
                A_BE[a,b,e] = ABE[a,b,e] / BE[b,e]
                
    print "P(A|B='T',E='F')="
    print A_BE[:,0,1]