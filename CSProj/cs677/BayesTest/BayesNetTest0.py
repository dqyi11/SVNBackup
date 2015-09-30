'''
Created on 2013-6-18

@author: Walter
'''

if __name__ == '__main__':
    
    from pylab import *
    import math
    
    B = [0.001,0.999] # P(B)
    E = [0.002,0.998] # P(E)
    
    A_BE = zeros((2,2,2),double) # P(C|B,D)
    
    A_BE[0,:,:] = [[0.95,0.94],[0.29,0.001]]
    A_BE[1,:,:] = [[0.05,0.06],[0.71,0.999]]
    
    J_A = array([[0.90, 0.05],[0.10, 0.95]]) # P(J|A)
    M_A = array([[0.70, 0.01],[0.30, 0.99]]) # P(M|A)
    
    BEAJM = zeros((2,2,2,2,2),double) # P(B,E,A,J,M)
    
    sum = 0.0
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        BEAJM[b,e,a,j,m] = math.log(J_A[j,a]) + math.log(M_A[m,a]) + math.log(A_BE[a,b,e]) + math.log(B[b]) + math.log(E[e])
                        BEAJM[b,e,a,j,m] = math.exp(BEAJM[b,e,a,j,m])
                        sum += BEAJM[b,e,a,j,m]
                        
    print sum
                        
    # case 1:
    J1 = zeros((2), double)
    M_J1 = zeros((2,2), double)
    A_MJ1 = zeros((2,2,2), double)
    B_A1 = zeros((2,2), double)
    E_A1 = zeros((2,2), double)
    AMJ1 = zeros((2,2,2), double)
    BA1 = zeros((2,2), double)
    EA1 = zeros((2,2), double)
    MJ1 = zeros((2,2),double)
    A1 = zeros((2), double)
    
    BEAJM1 = zeros((2,2,2,2,2),double)
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        J1[j] += BEAJM[b,e,a,j,m]
                        MJ1[m] += BEAJM[b,e,a,j,m]
                        AMJ1[a,m,j] += BEAJM[b,e,a,j,m]
                        BA1[b,a] += BEAJM[b,e,a,j,m]
                        EA1[e,a] += BEAJM[b,e,a,j,m]
                        A1[a] += BEAJM[b,e,a,j,m]
                        
    for m in range(2):
        for j in range(2):
            M_J1[m,j] = MJ1[m,j] / J1[j]
            
    for a in range(2):
        for m in range(2):
            for j in range(2):
                A_MJ1[a,m,j] = AMJ1[a,m,j] / MJ1[m,j]
    for a in range(2):
        for b in range(2):
            B_A1[b,a] = BA1[b,a] / A1[a]
    for a in range(2):
        for e in range(2):
            E_A1[e,a] = EA1[e,a] / A1[a]
    
    sum = 0.0                    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        BEAJM1[b,e,a,j,m] = math.log(B_A1[b,a]) + math.log(E_A1[e,a]) + math.log(A_MJ1[a,m,j]) + math.log(J1[j]) + math.log(M_J1[m,j])
                        BEAJM1[b,e,a,j,m] = math.exp(BEAJM1[b,e,a,j,m])
                        sum += BEAJM1[b,e,a,j,m]
    print sum
                        
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        if BEAJM1[b,e,a,j,m] != BEAJM[b,e,a,j,m]:
                            pass #print "NOT MATCH {} : {}".format(BEAJM1[b,e,a,j,m],BEAJM[b,e,a,j,m])
                       
    #case 2:
    B2 = zeros((2), double)
    E2 = zeros((2), double)
    A_BE2 = zeros((2,2,2), double)
    J_B2 = zeros((2,2), double)
    M_E2 = zeros((2,2), double)
    ABE2 = zeros((2,2,2), double)
    JB2 = zeros((2,2), double)
    ME2 = zeros((2,2), double)
    BE2 = zeros((2,2), double)
    
    BEAJM2 = zeros((2,2,2,2,2),double)
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        B2[b] += BEAJM[b,e,a,j,m]
                        E2[e] += BEAJM[b,e,a,j,m]
                        ABE2[a,b,e] += BEAJM[b,e,a,j,m]
                        JB2[j,b] += BEAJM[b,e,a,j,m]
                        ME2[m,e] += BEAJM[b,e,a,j,m]
                        BE2[b,e] += BEAJM[b,e,a,j,m]
                        
    for a in range(2):
        for b in range(2):
            for e in range(2):
                A_BE2[a,b,e] = ABE2[a,b,e] / BE2[b,e]
                 
    for m in range(2):
        for e in range(2):
            M_E2[m,e] = ME2[m,e] / E2[e]
            
    for j in range(2):
        for b in range(2):
            J_B2[j,b] = JB2[j,b] / B2[b]        
            
    sum = 0.0                    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        BEAJM2[b,e,a,j,m] = math.log(B2[b]) + math.log(E2[e]) + math.log(A_BE2[a,b,e]) + math.log(J_B2[j,b]) + math.log(M_E2[m,e])
                        BEAJM2[b,e,a,j,m] = math.exp(BEAJM2[b,e,a,j,m])
                        sum += BEAJM2[b,e,a,j,m]
    print sum
    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        if BEAJM2[b,e,a,j,m] != BEAJM[b,e,a,j,m]:
                            print "NOT MATCH {} : {}".format(BEAJM1[b,e,a,j,m],BEAJM[b,e,a,j,m])
                                           
    # case 3:
    B3 = zeros((2), double)
    E3 = zeros((2), double)
    A_BE3 = zeros((2,2,2), double)
    J_BE3 = zeros((2,2,2), double)
    M_BE3 = zeros((2,2,2), double)
    ABE3 = zeros((2,2,2), double)
    JBE3 = zeros((2,2,2), double)
    MBE3 = zeros((2,2,2), double)
    BE3 = zeros((2,2), double)
    
    BEAJM3 = zeros((2,2,2,2,2),double)
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        B3[b] += BEAJM[b,e,a,j,m]
                        E3[e] += BEAJM[b,e,a,j,m]
                        ABE3[a,b,e] += BEAJM[b,e,a,j,m]
                        JBE3[j,b,e] += BEAJM[b,e,a,j,m]
                        MBE3[m,b,e] += BEAJM[b,e,a,j,m]
                        BE3[b,e] += BEAJM[b,e,a,j,m]
                        
    for b in range(2):
        for e in range(2):
            for a in range(2):
                A_BE3[a,b,e] = ABE3[a,b,e] / BE3[b,e]
                
    for j in range(2):
        for b in range(2):
            for e in range(2):
                J_BE3[j,b,e] = JBE3[j,b,e] / BE3[b,e]
            
    for m in range(2):
        for b in range(2):
            for e in range(2):
                M_BE3[m,b,e] = MBE3[m,b,e] / BE3[b,e]
            
                        
    sum = 0.0                    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        BEAJM3[b,e,a,j,m] = math.log(B3[b]) + math.log(E3[e]) + math.log(A_BE3[a,b,e]) + math.log(J_BE3[j,b,e]) + math.log(M_BE3[m,b,e])
                        BEAJM3[b,e,a,j,m] = math.exp(BEAJM3[b,e,a,j,m])
                        sum += BEAJM3[b,e,a,j,m]
    print sum    
    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        if BEAJM3[b,e,a,j,m] != BEAJM[b,e,a,j,m]:
                            print "NOT MATCH {} : {}".format(BEAJM3[b,e,a,j,m],BEAJM[b,e,a,j,m])
    
    '''
    #case 4:
    B4 = zeros((2), double)
    E_B4 = zeros((2,2), double)
    A_BE4 = zeros((2,2,2), double)
    J_ABE4 = zeros((2,2,2,2), double)
    M_JABE4 = zeros((2,2,2,2,2),double)
    EB4 = zeros((2,2), double)
    ABE4 = zeros((2,2,2), double)
    JABE4 = zeros((2,2,2,2), double)
    MJABE4 = zeros((2,2,2,2,2),double)
    
    BEAJM4 = zeros((2,2,2,2,2),double)
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        B4[b] += BEAJM[b,e,a,j,m]
                        EB4[e,b] += BEAJM[b,e,a,j,m]
                        ABE4[a,b,e] += BEAJM[b,e,a,j,m]
                        JABE4[j,a,b,e] += BEAJM[b,e,a,j,m]                        
                        MJABE4[m,j,a,b,e] += BEAJM[b,e,a,j,m]
                        
    for e in range(2):
        for b in range(2):
            E_B4[e,b] = EB4[e,b] / B4[b]
            
    for b in range(2):
        for e in range(2):
            for a in range(2):
                A_BE4[a,b,e] = ABE4[a,b,e] / EB4[e,b]

    sum = 0.0
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    sum += JABE4[j,a,b,e]
                    J_ABE4[j,a,b,e] = JABE4[j,a,b,e] / ABE4[a,b,e]
    print sum
                       
    sum = 0.0
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        sum += MJABE4[m,j,a,b,e]
                        M_JABE4[m,j,a,b,e] = MJABE4[m,j,a,b,e] / JABE4[j,a,b,e]
    print sum
                     
    sum = 0.0                    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        BEAJM4[b,e,a,j,m] = math.log(B4[b]) + math.log(E_B4[e,b]) + math.log(A_BE4[a,b,e]) + math.log(J_ABE4[j,a,b,e]) + math.log(M_JABE4[m,j,a,b,e])
                        BEAJM4[b,e,a,j,m] = math.exp(BEAJM4[b,e,a,j,m])
                        sum += BEAJM4[b,e,a,j,m]
    print sum    
    
    for b in range(2):
        for e in range(2):
            for a in range(2):
                for j in range(2):
                    for m in range(2):
                        if BEAJM4[b,e,a,j,m] != BEAJM[b,e,a,j,m]:
                            print "NOT MATCH {} : {}".format(BEAJM4[b,e,a,j,m],BEAJM[b,e,a,j,m])
    '''