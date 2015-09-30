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
    
    AllJointProb = np.zeros((2,2,2,2,2,2,2))
    totalAll = 0.0
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            for g in range(2):
                                AllJointProb[a,b,c,d,e,f,g] = np.log(A[tuple([a])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(B[tuple([b, a])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(C[tuple([c, a, b, d])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(D[tuple([d, a])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(E[tuple([e, b, c, d])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(F[tuple([f, a, d, e])])
                                AllJointProb[a,b,c,d,e,f,g] += np.log(G[tuple([g, b, e])])
                                AllJointProb[a,b,c,d,e,f,g] = np.exp(AllJointProb[a,b,c,d,e,f,g])
                                totalAll += AllJointProb[a,b,c,d,e,f,g]
                                
    print totalAll
    
    CEABF = np.zeros((2,2,2,2,2))
    ABF = np.zeros((2,2,2))
    CEGFABD = np.zeros((2,2,2,2,2,2,2))
    ABD = np.zeros((2,2,2))
    ABCDEFG = np.zeros((2,2,2,2,2,2,2))
    EFG = np.zeros((2,2,2))
    ACEFBG = np.zeros((2,2,2,2,2,2))
    BG = np.zeros((2,2))   
    GBC = np.zeros((2,2,2))
    BC = np.zeros((2,2))
    BCDAE = np.zeros((2,2,2,2,2))
    AE = np.zeros((2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            for g in range(2):
                                CEABF[c,e,a,b,f] += AllJointProb[a,b,c,d,e,f,g]
                                ABF[a,b,f] += AllJointProb[a,b,c,d,e,f,g]
                                CEGFABD[c,e,g,f,a,b,d] += AllJointProb[a,b,c,d,e,f,g]
                                ABD[a,b,d] += AllJointProb[a,b,c,d,e,f,g]
                                ABCDEFG[a,b,c, d,e,f,g] += AllJointProb[a,b,c,d,e,f,g]
                                EFG[e,f,g] += AllJointProb[a,b,c,d,e,f,g]
                                ACEFBG[a,c,e,f,b,g] += AllJointProb[a,b,c,d,e,f,g]
                                BG[b,g] += AllJointProb[a,b,c,d,e,f,g]
                                GBC[g,b,c] += AllJointProb[a,b,c,d,e,f,g]
                                BC[b,c] += AllJointProb[a,b,c,d,e,f,g]
                                BCDAE[b,c,d,a,e] += AllJointProb[a,b,c,d,e,f,g]
                                AE[a,e] += AllJointProb[a,b,c,d,e,f,g]
    
    CE_ABF = np.zeros((2,2,2,2,2))                            
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for e in range(2):
                    for f in range(2):
                        CE_ABF[c,e,a,b,f] = CEABF[c,e,a,b,f] / ABF[a,b,f]
    
    print "P(C,E|A='T',B='T',F='F')="                   
    print CE_ABF[:,:,0,0,1]
    
    CEGF_ABD = np.zeros((2,2,2,2,2,2,2))
    ABCD_EFG = np.zeros((2,2,2,2,2,2,2))
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(2):
                            for g in range(2):
                                CEGF_ABD[c,e,g,f,a,b,d] = CEGFABD[c,e,g,f,a,b,d] / ABD[a,b,d]
                                ABCD_EFG[a,b,c,d,e,f,g] = ABCDEFG[a,b,c,d,e,f,g] / EFG[e,f,g]
    
    print "P(C,E,G,F|A='F',B='T',D='F')="                             
    print CEGF_ABD[:,:,:,:,1,0,1] 
    print "P(A,B,C,D|E='T',F='F',G='F')="
    print ABCD_EFG[:,:,:,:,0,1,1]
    
    ACEF_BG = np.zeros((2,2,2,2,2,2))
    for a in range(2):
        for c in range(2):
            for e in range(2):
                for f in range(2):
                    for b in range(2):
                        for g in range(2):
                            ACEF_BG[a,c,e,f,b,g] = ACEFBG[a,c,e,f,b,g] / BG[b,g]
    print "P(A,C,E,F|B='T',G='F')="                         
    print ACEF_BG[:,:,:,:,0,1]
    
    G_BC = np.zeros((2,2,2))
    for g in range(2):
        for b in range(2):
            for c in range(2):
                G_BC[g,b,c] = GBC[g,b,c] / BC[b,c]
    print "P(G|B='T',C='F')="            
    print G_BC[:,0,1]
    
    BCD_AE = np.zeros((2,2,2,2,2))
    for b in range(2):
        for c in range(2):
            for d in range(2):
                for a in range(2):
                    for e in range(2):
                        BCD_AE[b,c,d,a,e] = BCDAE[b,c,d,a,e] / AE[a,e]
    print "P(B,C,D|A='T',E='F')="          
    print BCD_AE[:,:,:,0,1]                    
     
    
    
    
    
    
                                 
                                                       
    
    