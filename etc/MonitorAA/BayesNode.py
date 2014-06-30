'''
Created on May 31, 2013

@author: walter
'''

from BayesNet import *
import numpy as np
import math
import scipy.stats

NodeType = {"DiscreteVar":0, "Normal":1, "Bernoulli":2, "Gamma":3, "Beta":4, 
            "Poisson":5, "InvGamma":6}
ValueType = ["Constant", "Variable", "Function"]

class DiscreteVarNode(object):
    
    def __init__(self, var, parents = None):
        self.var = var
        self.parentCnt = 0
        self.type = NodeType["DiscreteVar"]
        if isinstance(parents, str):
            self.parents = parents.split()
            self.parentCnt = len(self.parents)
        else:
            self.parents = None
            self.parentCnt = 0
        
        shape = 2 * np.ones(self.parentCnt+1, int)
        
        self.cpt = np.zeros(shape)
        self.children = []        
        #print (shape)
        #print (self.cpt)
        
    def __setitem__(self, pos, p):
        pos = list(pos)
        for i in range(len(pos)):
            if pos[i] == 'T':
                pos[i] = 0
            elif pos[i] == 'F':
                pos[i] = 1 
        pos = tuple(pos)     
        self.cpt[pos] = p               
        
    def __getitem__(self, pos):
        pos = list(pos)
        for i in range(len(pos)):
            if pos[i] == 'T':
                pos[i] = 0
            elif pos[i] == 'F':
                pos[i] = 1 
        pos = tuple(pos)   
        return self.cpt[pos]
    
    def hasParent(self, var):
        if self.parents != None:
            for p in self.parents:
                if p == var:
                    return True
        return False   
    
    def getName(self):
        pass
    
    def getRandomInitSample(self):        
        if np.random.random() >= 0.5:
            return "F"
        else:
            return "T"
    
    def completeFrom(self, refVal):       
        if refVal == True:
            self.cpt[1] = np.ones(np.ones(self.parentCnt+1, int)) - self.cpt[0]
        else:
            self.cpt[0] = np.ones(np.ones(self.parentCnt+1, int)) - self.cpt[1]    
        #print self.cpt
    
    def printCPT(self):
        Trans = "True", "False"
        text = ""
        if self.parentCnt == 0:
            for val in range(2):
                text += "P({}={})={}\\n".format(self.var, Trans[val], self.cpt[val])    
        else:   
            for i in range(2**(self.parentCnt)):
                for val in range(2):
                    text += "P({}={}|".format(self.var, Trans[val])
                    pos = [val]
                    nbr = i
                    for j in range(self.parentCnt):
                        idx = int(nbr/(2**(self.parentCnt-j-1)))
                        nbr -= idx * (2**(self.parentCnt-j-1))
                        text += "{}={}".format(self.parents[j], Trans[idx])
                        if j < self.parentCnt-1:
                            text += ","
                        else:
                            text += ")"         
                        pos.append(idx)
                    pos = tuple(pos)           
                    text += "={}\\n".format(self.cpt[pos])
     
        return text

class BayesNode(object):
    
    def hasParent(self, var):
        if self.parents != None:
            for p in self.parents:
                if p == var:
                    return True
        return False 

class NormalNode(BayesNode):
    
    def __init__(self, var, mean, variance, meanFunc=None, varianceFunc=None):
        self.var = var
        self.mean = mean
        self.meanType = ValueType[0]
        self.variance = variance
        self.varianceType = ValueType[0]
        self.type = NodeType["Normal"]
        self.parents = []
        self.meanFunc = meanFunc
        self.varianceFunc = varianceFunc
        self.children = []
        
        if meanFunc == None:
            if isinstance(mean, str):
                self.parents.append(mean)
                self.meanType = ValueType[1]
        else:
            self.meanType = ValueType[2]
            for m in mean:
                if isinstance(m, str):
                    self.parents.append(m)
                
        
        if varianceFunc == None:
            if isinstance(variance, str):
                self.parents.append(variance)
                self.varianceType = ValueType[1]
        else:
            self.varianceType = ValueType[2]
            for v in variance:
                if isinstance(v, str):
                    self.parents.append(v)
                
            
            
    def getLogLikelihood(self, val, instance):    
        meanVal = 0.0
        varianceVal = 0.0
        if self.meanType == ValueType[0]:
            meanVal = self.mean
        elif self.meanType == ValueType[1]:
            meanVal = instance[self.mean]
        elif self.meanType == ValueType[2]:
            params = []
            for m in self.mean:
                if isinstance(m, str):
                    params.append(instance[m])
                else:
                    params.append(m)
            meanVal = self.meanFunc(*tuple(params))        
            
        if self.varianceType == ValueType[0]:
            varianceVal = self.variance
        elif self.varianceType == ValueType[1]:
            varianceVal = instance[self.variance]
        elif self.varianceType == ValueType[2]:
            params = []
            for v in self.variance:
                if isinstance(v, str):
                    params.append(instance[v])
                else:
                    params.append(v)
            varianceVal = self.varianceFunc(*tuple(params))            
        
        '''       
        logLikelihood = -(val - meanVal)**2 / (2*varianceVal) - 0.5 * np.log(varianceVal)
        '''
        if varianceVal <= 0:
            return 0.0
            
        rv = scipy.stats.norm(loc=meanVal, scale=varianceVal**0.5)
        logLikelihood = np.log(rv.pdf(val))
        
        return logLikelihood
       
        
class BernoulliNode(BayesNode):
    
    def __init__(self, var, prob, probFunc = None):
        self.var = var
        self.prob = prob
        self.probType = ValueType[0]
        self.type = NodeType["Bernoulli"]
        self.parents = []
        self.probFunc = probFunc
        self.children = []
        
        if probFunc == None:
            if isinstance(prob, str):
                self.parents.append(prob)
                self.probType = ValueType[1]
        else:
            self.probType = ValueType[2]
            for m in prob:
                if isinstance(m, str):
                    self.parents.append(m)
        
        
            
    def getProbability(self, instance):
        probVal = 0.0
        if self.probType == ValueType[0]:
            probVal = self.prob
        elif self.probType == ValueType[1]:
            probVal = instance[self.prob]
        elif self.probType == ValueType[2]:
            params = []
            for p in self.prob:
                if isinstance(p, str):
                    params.append(instance[p])
                else:
                    params.append(p)
            probVal = self.probFunc(*tuple(params))   
            
        if probVal > 1:
            probVal = 1.0
        elif probVal < 0:
            probVal = 0.0
        return probVal
    
    def getLogLikelihood(self, val, instance): 
        probVal = 0.0
        if self.probType == ValueType[0]:
            probVal = self.prob
        elif self.probType == ValueType[1]:
            probVal = instance[self.prob]
        elif self.probType == ValueType[2]:
            params = []
            for p in self.prob:
                if isinstance(p, str):
                    params.append(instance[p])
                else:
                    params.append(p)
            probVal = self.probFunc(*tuple(params))            
            
        logLikelihood = np.log(probVal)
        
        return logLikelihood
        
class GammaNode(BayesNode):
    
    def __init__(self, var, alpha, beta, alphaFunc = None, betaFunc = None):
        self.var = var
        self.alpha = alpha
        self.alphaType = ValueType[0]
        self.beta = beta
        self.betaType = ValueType[0]
        self.type = NodeType["Gamma"]
        self.parents = []
        self.alphaFunc = alphaFunc
        self.betaFunc = betaFunc
        self.children = []
        
        if alphaFunc == None:
            if isinstance(alpha, str):
                self.parents.append(alpha)
                self.alphaType = ValueType[1]
        else:
            self.alphaType = ValueType[2]
            for s in alpha:
                if isinstance(s, str):
                    self.parents.append(s)
        if betaFunc == None:
            if isinstance(beta, str):
                self.parents.append(beta)  
                self.betaType = ValueType[1]
        else:
            self.betaType = ValueType[2]
            for i in beta:
                if isinstance(i, str):
                    self.parents.append(i)
                    
    def getLogLikelihood(self, val, instance):    
        alphaVal = 0.0
        betaVal = 0.0
        if self.alphaType == ValueType[0]:
            alphaVal = self.alpha
        elif self.alphaType == ValueType[1]:
            alphaVal = instance[self.alpha]
        elif self.alphaType == ValueType[2]:
            params = []
            for i in self.alpha:
                if isinstance(i, str):
                    params.append(instance[i])
                else:
                    params.append(i)
            alphaVal = self.alphaFunc(*tuple(params)) 
            
        if self.betaType == ValueType[0]:
            betaVal = self.beta
        elif self.betaType == ValueType[1]:
            betaVal = instance[self.beta]
        elif self.betaType == ValueType[2]:
            params = []
            for i in self.beta:
                if isinstance(i, str):
                    params.append(instance[i])
                else:
                    params.append(i)
            betaVal = self.betaFunc(*tuple(params))    
          
        '''    
        logLikelihood =  (shapeVal - 1) * np.log(val) - invscaleVal * val
        print self.var
        print shapeVal
        print invscaleVal
        print val
        
        logLikelihood += np.log(invscaleVal**shapeVal / math.gamma(shapeVal))
        '''
        rv = scipy.stats.gamma(alphaVal, loc = .0, scale = 1.0/betaVal)
        logLikelihood = np.log(rv.pdf(val))
        
        return logLikelihood
            
class InvGammaNode(BayesNode):
    
    def __init__(self, var, alpha, beta, alphaFunc=None, betaFunc=None):
        self.var = var
        self.alpha = alpha  
        self.alphaType = ValueType[0]
        self.beta = beta
        self.betaType = ValueType[0]
        self.type = NodeType["InvGamma"]
        self.parents = []
        self.alphaFunc = alphaFunc
        self.betaFunc = betaFunc
        self.children = []
        
        if alphaFunc == None:
            if isinstance(alpha, str):
                self.parents.append(alpha)
                self.alphaType = ValueType[1]
        else:
            self.alphaType = ValueType[2]
            for s in alpha:
                if isinstance(s, str):
                    self.parents.append(s)            
            
        if betaFunc == None:    
            if isinstance(beta, str):
                self.parents.append(beta)  
                self.invscaleType = ValueType[1]
        else:
            self.betaType = ValueType[2]
            for i in beta:
                if isinstance(i, str):
                    self.parents.append(i)
            
    def getLogLikelihood(self, val, instance):    
        alphaVal = 0.0
        betaVal = 0.0
        if self.alphaType == ValueType[0]:
            alphaVal = self.alpha
        elif self.alphaType == ValueType[1]:
            alphaVal = instance[self.alpha]
        elif self.alphaType == ValueType[2]:
            params = []
            for i in self.alpha:
                if isinstance(i, str):
                    params.append(instance[i])
                else:
                    params.append(i)
            alphaVal = self.alphaFunc(*tuple(params)) 
            
        if self.betaType == ValueType[0]:
            betaVal = self.beta
        elif self.betaType == ValueType[1]:
            betaVal = instance[self.beta]
        elif self.betaType == ValueType[2]:
            params = []
            for i in self.beta:
                if isinstance(i, str):
                    params.append(instance[i])
                else:
                    params.append(i)
            betaVal = self.betaFunc(*tuple(params))    
        
        '''       
        logLikelihood =  - (shapeVal+1) * np.log(val) - invscaleVal / val
        logLikelihood + shapeVal * np.log(invscaleVal) - np.log(math.gamma(shapeVal))
        '''

        rv = scipy.stats.invgamma(alphaVal, loc = .0, scale = betaVal)
        logLikelihood = np.log(rv.pdf(val))
        
        return logLikelihood
            
        
class BetaNode(BayesNode):
    
    def __init__(self, var, alpha, beta, alphaFunc=None, betaFunc=None):
        self.var = var
        self.alpha = alpha
        self.alphaType = ValueType[0]
        self.beta = beta
        self.betaType = ValueType[0]
        self.type = NodeType["Beta"]
        self.parents = []
        self.alphaFunc = alphaFunc
        self.betaFunc = betaFunc
        self.children = []
        
        if alphaFunc == None:
            if isinstance(alpha, str):
                self.parents.append(alpha)
                self.alphaType = ValueType[1]
        else:
            self.alphaType = ValueType[2]
            for s in alpha:
                if isinstance(s, str):
                    self.parents.append(s)           
            
        if betaFunc == None:    
            if isinstance(beta, str):
                self.parents.append(beta)  
                self.betaType = ValueType[1]
        else:
            self.betaType = ValueType[2]
            for i in beta:
                if isinstance(i, str):
                    self.parents.append(i)
            
    def getLogLikelihood(self, val, instance):    
        alphaVal = 0.0
        betaVal = 0.0
        if self.alphaType == ValueType[0]:
            alphaVal = self.alpha
        elif self.alphaType == ValueType[1]:
            alphaVal = instance[self.alpha]
        elif self.alphaType == ValueType[2]:
            params = []
            for i in self.alpha:
                if isinstance(i, str):
                    params.append(instance[i])
                else:
                    params.append(i)
            alphaVal = self.alphaFunc(*tuple(params)) 
            
        if self.betaType == ValueType[0]:
            betaVal = self.beta
        elif self.betaType == ValueType[1]:
            betaVal = instance[self.beta]
        elif self.betaType == ValueType[2]:
            params = []
            for i in self.beta:
                if isinstance(i, str):
                    params.append(instance[i])
                else:
                    params.append(i)
            betaVal = self.betaFunc(*tuple(params))   
        
        '''
        if alphaVal == 1 and betaVal ==1:
            logLikelihood = 0.0
        elif alphaVal == 1:
            logLikelihood = (betaVal - 1) * np.log(1-val)
        elif betaVal == 1:
            logLikelihood = (alphaVal - 1) * np.log(val)    
        else:    
            logLikelihood = (alphaVal - 1) * np.log(val) + (betaVal -1) * np.log(1-val)
        
        logLikelihood += np.log(math.gamma(alphaVal+betaVal)) - np.log(math.gamma(alphaVal))
        logLikelihood -= np.log(math.gamma(betaVal))
        '''
        rv = scipy.stats.beta(alphaVal, betaVal)
        logLikelihood = np.log(rv.pdf(val))        
        
        return logLikelihood
    
        
class PoissonNode(BayesNode):
    
    def __init__(self, var, rate, rateFunc=None):
        self.var = var
        self.rate = rate
        self.rateType = ValueType[0]
        self.type = NodeType["Poisson"]
        self.parents = []
        self.rateFunc = rateFunc
        self.children = []
        
        if self.rateFunc == None:
            if isinstance(rate, str):
                self.parents.append(rate)
                self.rateType = ValueType[1]
        else:
            self.rateType = ValueType[2]
            for r in rate:
                if isinstance(r, str):
                    self.parents.append(r)    
            
    def getLogLikelihood(self, val, instance):
        rateVal = 0.0
        if self.rateType == ValueType[0]:
            rateVal = self.rate
        elif self.rateType == ValueType[1]:
            rateVal = instance[self.rate]
        elif self.alphaType == ValueType[2]:
            params = []
            for r in self.rate:
                if isinstance(r, str):
                    params.append(instance[r])
                else:
                    params.append(r)
            rateVal = self.rateFunc(*tuple(params)) 
            
        '''
        logLikelihood = rateVal * np.log(val) - np.log(math.gamma(val+1)) - rateVal
        '''
        
        rv = scipy.stats.poisson(rateVal)        
        logLikelihood = np.log(rv.pmf(val))
        
        return logLikelihood
