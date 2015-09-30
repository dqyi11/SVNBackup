'''
Created on May 28, 2013

@author: walter
'''

if __name__ == '__main__':
    
    import numpy as np
    from BayesNet import *
    
    Net = BayesNet()        
    B = BernoulliNode("Burglary")          
    E = BernoulliNode("Earthquake")
    A = BernoulliNode("Alarm", 'Burglary Earthquake')
    J = BernoulliNode("JohnCalls", 'Alarm')
    M = BernoulliNode("MaryCalls", 'Alarm')
    
    B['T'] = 0.001
    E['T'] = 0.002
    A['T','T','T'] = 0.95
    A['T','T','F'] = 0.94
    A['T','F','T'] = 0.29
    A['T','F','F'] = 0.001
    J['T','T'] = 0.90
    J['T','F'] = 0.05
    M['T','T'] = 0.70
    M['T','F'] = 0.01
    
    Net.addNode(B)
    Net.addNode(E)
    Net.addNode(A)
    Net.addNode(J)
    Net.addNode(M)

    Net.completeFrom(True)
    
    AllJointProb = np.zeros((2,2,2,2,2))
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        AllJointProb[b,e,a,j,m] = np.log(B[tuple([b])])
                        AllJointProb[b,e,a,j,m] += np.log(E[tuple([e])])
                        AllJointProb[b,e,a,j,m] += np.log(A[tuple([a,b,e])])
                        AllJointProb[b,e,a,j,m] += np.log(J[tuple([j,a])])
                        AllJointProb[b,e,a,j,m] += np.log(M[tuple([m,a])])
                        AllJointProb[b,e,a,j,m] = np.exp(AllJointProb[b,e,a,j,m])
                        
    BJM = np.zeros((2,2,2))
    AJM = np.zeros((2,2,2))
    EJM = np.zeros((2,2,2))
    JM = np.zeros((2,2))
    BJ = np.zeros((2,2))
    BM = np.zeros((2,2))
    J = np.zeros((2))
    M = np.zeros((2))
    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        BJM[b,j,m] += AllJointProb[b,e,a,j,m]
                        JM[j,m] += AllJointProb[b,e,a,j,m]
                        AJM[a,j,m] += AllJointProb[b,e,a,j,m]
                        EJM[e,j,m] += AllJointProb[b,e,a,j,m]
                        BJ[b,j] += AllJointProb[b,e,a,j,m]
                        BM[b,m] += AllJointProb[b,e,a,j,m]
                        J[j] += AllJointProb[b,e,a,j,m]
                        M[m] += AllJointProb[b,e,a,j,m]
              
    B_JM = np.zeros((2,2,2))     
    for b in range(2):
        for j in range(2):
            for m in range(2):
                B_JM[b,j,m] = BJM[b,j,m] / JM[j,m]
                
    totalJM = 0.0
    for j in range(2):
        for m in range(2):
            totalJM += JM[j,m]
            
    print totalJM
                
    print "P(B|J='T',M='T')="
    print B_JM[:,0,0]
    
    A_JM = np.zeros((2,2,2))
    for a in range(2):
        for j in range(2):
            for m in range(2):
                A_JM[a,j,m] = AJM[a,j,m] / JM[j,m]
                
    print "P(A|J='T',M='T')="
    print A_JM[:,0,0]
    
    E_JM = np.zeros((2,2,2))
    for e in range(2):
        for j in range(2):
            for m in range(2):
                E_JM[e,j,m] = EJM[e,j,m] / JM[j,m]
                
    print "P(E|J='T',M='T')="
    print E_JM[:,0,0]
    
    print "P(B|J='F',M='F')="
    print B_JM[:,1,1]
    
    print "P(B|J='T',M='F')="
    print B_JM[:,0,1]
    
    B_J = np.zeros((2,2))
    for b in range(2):
        for j in range(2):
            B_J[b,j] = BJ[b,j]/J[j]
    
    print "P(B|J='T')="
    print B_J[:,0]  
    
    B_M = np.zeros((2,2))
    for b in range(2):
        for m in range(2):
            B_M[b,m] = BM[b,m]/M[m]
    
    print "P(B|M='T')="
    print B_M[:,0]
                
                        