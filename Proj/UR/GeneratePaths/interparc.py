import numpy as np
import scipy.interpolate as sp
import math
import csv

def diffCOL(matrix):
    newMAT = []
    newROW = []
    for i in range(len(matrix)-1):
        for j in range(len(matrix[i])):
            diff = matrix[i+1][j]-matrix[i][j]
            newROW.append(diff) 
        newMAT.append(newROW)
        newROW = []
    #Stack the matrix to get xyz in columns
    newMAT = np.vstack(newMAT)
    return newMAT
    
def squareELEM(matrix):            
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            matrix[i][j] = matrix[i][j]*matrix[i][j]
    return matrix
        
def sumROW(matrix):      
    newMAT = []
    for j in range(len(matrix)):
        rowSUM = 0
        for k in range(len(matrix[j])):
            rowSUM = rowSUM + matrix[j][k]
        newMAT.append(rowSUM)  
    return newMAT
            
def sqrtELEM(matrix):
    for i in range(len(matrix)):
        matrix[i] = math.sqrt(matrix[i])
    return matrix      
    
def sumELEM(matrix):
    sum = 0
    for i in range(len(matrix)):
        sum = sum + matrix[i]
    return sum
    
def diffMAT(matrix,denom):
    newMAT = []
    for i in range(len(matrix)):
        newMAT.append(matrix[i]/denom)
    return newMAT
    
def cumsumMAT(matrix):
    first = 0
    newmat = []
    newmat.append(first)
    #newmat.append(matrix)
    for i in range(len(matrix)):
        newmat.append(matrix[i])
    cum = 0
    for i in range(len(newmat)):
        cum = cum + newmat[i] 
        newmat[i] = cum
    return newmat
        
def divMAT(A,B):
    newMAT = []
    for i in range(len(A)):
        newMAT.append(A[i] / B[i])
    return newMAT

def minusVector(t,cumarc):
    newMAT = []
    for i in range(len(t)):
        newMAT.append(t[i] - cumarc[i])
    return newMAT
    
def replaceIndex(A,B):
    newMAT = []
    #print "A: ,",A
    #print "LENA: ",len(A)
    #print "B: ,",B
    #print "LENB: ",len(B)
    for i in range(len(B)):
        index = B[i]
        newMAT.append(A[index])
    return newMAT        
   
def matSUB(first,second):
    newMAT = []
    newCOL = []
    for i in range(len(first)):
        for j in range(len(first[i])):
            newMAT.append(first[i][j] - second[i][j])
        #newMAT.append(newCOL)
    return newMAT
    
def matADD(first,second):
    newMAT = []
    newCOL = []
    #print "F: ",first
    #print "S: ",second
    for i in range(len(first)):
        for j in range(len(first[i])):
            newMAT.append(first[i][j]+second[i][j])
        #newMAT.append(newCOL)
    return newMAT
    
def matMULTI(first,second):
    """
    Take in two matrix
    multiply each element against the other at the same index
    return a new matrix
    """
    #print "FIRST: ",first
    #print "SECOND: ",second
    newMAT = []
    newCOL = []
    for i in range(len(first)):
        for j in range(len(first[i])):
            newMAT.append(first[i][j]*second[i][j])
        #newMAT.append(newCOL)
    return newMAT 
    
def matDIV(first,second):
    """
    Take in two matrix
    multiply each element against the other at the same index
    return a new matrix
    """
    newMAT = []
    newCOL = []
    for i in range(len(first)):
        for j in range(len(first[i])):
            newMAT.append(first[i][j]/second[i][j])
        #newMAT.append(newCOL)
    return newMAT

def vecDIV(first,second):
    """
    Take in two arrays
    multiply each element against the other at the same index
    return a new array
    """
    newMAT = []
    for i in range(len(first)):
        newMAT.append(first[i]/second[i])
    return newMAT

def replaceROW(matrix,replacer,adder):
    newMAT = []
    #print "MAT: ",matrix
    #print "REP: ",replacer
    if adder != 0:
        for i in range(len(replacer)):
            newMAT.append(matrix[replacer[i]+adder])
    else:
        for i in range(len(replacer)):
            newMAT.append(matrix[replacer[i]])
    return np.vstack(newMAT)
            
            
def interparc(t,px,py,*args):
    inputs = [t,px,py,args]
    #print "T: ",t
    #print "X: ",px
    #print "Y: ",py
    #print "Z: ",varargin
    
    #If we dont get at least a t, x, and y value we error
    if len(inputs)<3:
        print "ERROR: NOT ENOUGH ARGUMENTS"
        
    #Should check to make sure t is a single integer greater than 1
    t = t
    if (t>1) and (t%1==0):
        t = np.linspace(0,1,t)
    elif t<0 or t>1:
        print "Error: STEP SIZE t IS NOT ALLOWED"
       
    nt = len(t)
    
    px = px
    py = py
    n = len(px)
    
    if len(px) != len(py):
        print "ERROR: MUST BE SAME LENGTH"
    elif n < 2:
        print "ERROR: MUST BE OF LENGTH 2"
        
    pxy = [px,py]
    #pxy = np.transpose(pxy)
    ndim = 2
    
    method = 'linear'
    
    if len(args) > 1:
        if isinstance(args[len(args)-1], basestring) == True:
            method = args[len(args)-1]
            if method != 'linear' and method != 'pchip' and method != 'spline':
                print "ERROR: INVALID METHOD"
    elif len(args)==1:
        method = args[0]
    #print method
    method = 'linear'
    # Try to append all the arguments together
    for i in range(len(args)):
        if isinstance(args[i], basestring) != True:
            pz = args[i]
            print "Z",len(pz)
            if len(pz) != n:
                print "ERROR: LENGTH MUST BE SAME AS OTHER INPUT"
            pxy.append(pz)
    #print "DIM LEN: ",len(pxy)
    ndim = len(pxy)
    
    pt = np.zeros((nt,ndim))
#Check for rounding errors here
    # Transpose the matrix to align with matlab codes method
    pxy = np.transpose(pxy)
    chordlen = sqrtELEM(sumROW(squareELEM(diffCOL(pxy))))
    chordlen = diffMAT(chordlen,sumELEM(chordlen))
    cumarc = cumsumMAT(chordlen)
    '''    
    print ""
    print ""
    print "Third operation (CUMARC): \n",np.vstack(cumarc)
    print ""
    '''

    if method == 'linear':
        inter = np.histogram(bins=t,a=cumarc)
        tbins = inter[1]
        hist= inter[0]
        tbinset=[]
        index=0
        tbinset.append(index)
        '''
        print "ORIGINAL TBIN: \n",hist
        print "LEN: \n",len(hist)
        print ""
        '''
        for i in range(len(hist)):
            if hist[i]>0:
                index=index+hist[i]
                tbinset.append(index)
            else:
                tbinset.append(index)

        for i in range(len(tbinset)):
            if tbinset[i]<= 0 or t[i]<=0:
                tbinset[i] = 1
            elif tbinset[i]>=n or t[i]>=1:
                tbinset[i] = n-1
        '''        
        print ""
        print "TBINSET: ",tbinset
        print "LEN: ",len(tbinset)
        '''
        #Take off one value to match the way matlab does indexing
        for i in range(len(tbinset)):
            tbinset[i]=tbinset[i]-1

        s = divMAT(minusVector(t,replaceIndex(cumarc,tbinset)),replaceIndex(chordlen,tbinset) )

        #Breakup the parts of pt
        repmat = np.transpose(np.reshape(np.vstack(np.tile(s,(1,ndim))[0]),(ndim,-1)))
        sub = np.reshape( np.vstack( matSUB( replaceROW(pxy,tbinset,1) , replaceROW(pxy,tbinset,0) ) ) , (-1,ndim) )
        multi = np.reshape( np.vstack(matMULTI( sub , repmat )) ,(-1,ndim) )
        pt = np.reshape( np.vstack( matADD( replaceROW(pxy,tbinset,0) , multi ) ) ,(-1,ndim) )
        return pt