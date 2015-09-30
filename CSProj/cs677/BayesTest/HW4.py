'''
Created on 2013-5-14

@author: Walter
'''

if __name__ == '__main__':
    
    from pylab import *
    
    A = [0.1,0.9] # P(A)
    B_A = array([[0.2,0.4],[0.3,0.5],[0.5,0.1]]) # P(B|A)
    C_BD = zeros((2,3,2),double) # P(C|B,D)
    
    C_BD[0,:,:] = [[0.6,0.7],[0.8,0.9],[0.11,0.21]]
    C_BD[1,:,:] = [[0.4,0.3],[0.2,0.1],[0.89,0.79]]
    D = [0.31,0.69] # P(D)
    
    # ABCD[a,b,c,d] = C_BDA(c,b,d,a)*D_BA(d,b,a)*B_A(b,a)*A(a)
    # = C_BD(c,b,d) * D(d) * B_A(b,a) * A(a)
    
    ABCD = zeros((2,3,2,2),double) # P(A,B,C,D)
    
    for a in range(2):
        for b in range(3):
            for c in range(2):
                for d in range(2):
                    ABCD[a,b,c,d] = C_BD[c,b,d] *D[d] *B_A[b,a] *A[a]
                   
    CD = zeros((2,2),double)
    BCD = zeros((3,2,2),double)
    for a in range(2):
        for b in range(3):
            for c in range(2):
                for d in range(2):
                    CD[c,d] = CD[c,d]+ABCD[a,b,c,d]
                    BCD[b,c,d] = BCD[b,c,d] + ABCD[a,b,c,d]
    
    B_CD = zeros((3,2,2),double)
    for b in range(3):
        for c in range(2):
            for d in range(2):
                B_CD[b,c,d] = BCD[b,c,d]/CD[c,d]
                
    print "p(B=1|C=t,D=f) = {}".format(B_CD[0,0,1])
    print "p(B=1|C=t,D=t) = {}".format(B_CD[0,0,0])
    
    #VALIDATE
    total = 0
    for c in range(2):
        for d in range(2):
            total = total + CD[c,d]
    print "sum(p(C,D) = {}".format(total)
           
    total = 0
    for b in range(3):
        for c in range(2):
            for d in range(2):
                total = total + BCD[b,c,d]
    print "sum(p(B,C,D) = {}".format(total)
    
    total = 0
    for a in range(2):
        for b in range(3):
            for c in range(2):
                for d in range(2):
                    total = total + ABCD[a,b,c,d]
    print "sum(p(A,B,C,D) = {}".format(total)
    
    for a in range(2):
        for b in range(3):
            print "B_A[{},{}]={}".format(a,b,B_A[b,a])
            
    
    for c in range(2):
        for b in range(3):
            for d in range(2):
                print "C_BD[{},{},{}]={}".format(c,b,d,C_BD[c,b,d])
    
    
    totals = zeros((2,2), double)
    for b in range(3):
        for c in range(2):
            for d in range(2):
                #print "BCD[{},{},{}]={}".format(b,c,d,BCD[b,c,d])
                #print "CD[{},{}]={}".format(c,d,CD[c,d])
                print "B_CD[{},{},{}]={}".format(b,c,d,B_CD[b,c,d]) 
                totals[c,d] += B_CD[b,c,d]
                
    print totals
                
       
    