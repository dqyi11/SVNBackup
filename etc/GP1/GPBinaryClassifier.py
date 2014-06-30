'''
Created on Feb 16, 2014

@author: daqing_yi
'''

import numpy as np
import scipy.special as sp

class GPBinaryClassifier(object):
    '''
    classdocs
    '''

    def setCovFunc(self, covFunc ):
        
        self.covFunc = covFunc
        
    def setOptParam(self, maxIt, optTol):
        
        self.maxIt = maxIt
        self.optTol = optTol
        
    def setPriorVar(self, priorVar):
        
        self.priorVar = priorVar
        
    def avSigGaussian(self, mn, vr):
        
        erflambda = np.sqrt(np.pi / 4.0)
        erflambda2 = erflambda**2
        v = erflambda * mn / np.sqrt(1 + 2 * erflambda2 * vr)

        return 0.5 + 0.5 * sp.erf(v)
        
    def learn(self, xtrain, ctrain):
        self.xtrain = xtrain
        self.ctrain = ctrain
        self.trainDataSize = xtrain.shape[0]
        self.dim = xtrain.shape[1]
        
        xt = np.matrix(self.xtrain)
        ct = np.matrix(self.ctrain)
        
        self.Kxx = np.matrix(self.covFunc.K( xtrain ))
        self.Kxx = self.Kxx + np.matrix(np.eye(self.trainDataSize)) * self.priorVar
        
        y = np.matrix( np.zeros( (self.trainDataSize, 1) ) )
        self.D = np.matrix( np.zeros((self.trainDataSize, self.trainDataSize)) )
        self.sigvec = np.matrix( np.zeros( (self.trainDataSize, 1) ) )
        
        print self.Kxx.shape
        
        # Newton update for Laplace approximation
        for t in range(self.maxIt):
            
            yold = y
            
            self.sigvec = 1.0 / (1.0 + np.exp(- y))
            
            for i in range(self.trainDataSize):
                self.D[i,i] = self.sigvec[i,0] * (1 - self.sigvec[i,0])
                
            temp1 = self.D * y + ct - self.sigvec
            temp2 = np.eye( self.trainDataSize ) + self.D * self.Kxx
            

            y = self.Kxx * np.array(np.linalg.inv(np.matrix(temp2))) * temp1           

            delta = np.linalg.norm( np.matrix(y - yold) )
            print delta
            if delta < self.optTol:
                break
            
        #print sigvec    
        #meantrain= np.matrix( np.zeros((trainDataSize,1)) )
        self.meantrain = self.sigvec
        #print self.ctrain.shape
        #print self.sigvec.shape
        self.cms = self.ctrain - self.sigvec 
        
    def predict(self, xtest):
               
        testDataSize = xtest.shape[0]

        newX = np.array(np.vstack((self.xtrain, xtest)))
        
        K = np.matrix( self.covFunc.K(newX) )
        
        print K[self.trainDataSize:self.trainDataSize+testDataSize, self.trainDataSize:self.trainDataSize+testDataSize].shape

        # compute prediction
        meantest = np.matrix( np.zeros((testDataSize,1)) )
               
        invD = np.linalg.inv(self.D)
        Kx1x1 =K[self.trainDataSize:self.trainDataSize+testDataSize, self.trainDataSize:self.trainDataSize+testDataSize]
        Kx1x = K[self.trainDataSize:self.trainDataSize+testDataSize, 0:self.trainDataSize]
        
        #print Kx1x1.shape
        #print Kxx1.shape
        #print Kx1x.shape
        
        for n in range(testDataSize):
            #print Kx1x[n, :].shape
            #print self.cms.shape
            
            #print Kx1x[n, :].shape
            #print self.cms.shape
            
            mn = Kx1x[n, :] * self.cms
            
            #print Kx1x1[n,n].shape
            #print Kx1x[n,:].shape
            #print Kx1x[n,:].T.shape
            
            #print Kx1x[n,:]
            #print Kx1x[n,:].T
            
            vr = Kx1x1[n, n] - Kx1x[n, :] * ( np.linalg.inv( self.Kxx + invD ) ) * Kx1x[n, :].T
            
            #print mn.shape
            #print vr.shape
            
            meantest[n,:] = self.avSigGaussian(mn, vr)
        
        # training data log likelihood
        
        #print meantest
    
        return meantest        