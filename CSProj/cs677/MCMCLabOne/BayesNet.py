'''
Created on 2013-5-25

@author: Walter
'''

import numpy as np
import pygraphviz as pgv
import copy
import matplotlib
matplotlib.use("WXAgg")
import matplotlib.pyplot as plt


NodeType = ["Bernoulli"]

class BernoulliNode(object):
    
    def __init__(self, var, parents = None):
        self.var = var
        self.parentCnt = 0
        self.type = NodeType[0]
        if isinstance(parents, str):
            self.parents = parents.split()
            self.parentCnt = len(self.parents)
        else:
            self.parents = None
            self.parentCnt = 0
        
        shape = 2 * np.ones(self.parentCnt+1, int)
        
        self.cpt = np.zeros(shape)        
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
   

class BayesNet(object):

    def __init__(self):
        '''
        Constructor
        '''
        self.nodeList = []
        
    def addNode(self, node):
        self.nodeList.append(node)    
        
    def find(self, nodeName):
        for n in self.nodeList:
            if n.var == nodeName:
                return n
        return None

    def getParents(self, nodeName):
        parents = []        
        n = self.find(nodeName)
        if n.parents != None:
            parents = n.parents
        return parents
    
    def getChildren(self, nodeName):
        children = []
        for c in self.nodeList:
            if c.hasParent(nodeName):
                children.append(c.var)
        
        return children
    
    def getParentsOfChildren(self, nodeName):
        
        parentsOfChildren = []
        children = self.getChildren(nodeName)
        
        for c in children:
            parents = self.getParents(c)
            for p in parents:
                if (p in parentsOfChildren) == False and p != nodeName:
                    parentsOfChildren.append(p)           
         
        return parentsOfChildren        
    
    def getMarkovBlanket(self, nodeName):
        
        markovBlanket = list(set(self.getParents(nodeName)).union(set(self.getChildren(nodeName))))
        return list(set(markovBlanket).union(set(self.getParentsOfChildren(nodeName))))
    
    def completeFrom(self, refVal):
        for n in self.nodeList:
            n.completeFrom(refVal)
    
    def plot(self, filename = None):
        
        g = pgv.AGraph(directed=True)
        
        for n in self.nodeList:
            g.add_node(n.var)
            g.add_node(n.printCPT(), shape='box')
                    
        for n in self.nodeList:
            if n.parents != None:
                for p in n.parents:
                    g.add_edge(p, n.var)

        g.layout()
        outputFilename = 'file.png'
        if isinstance(filename, str):
            outputFilename = filename + '.png'
            
        #g.draw(outputFilename)
        g.write("net.dot")
          
class Query(object):

    def __init__(self, queryVars, evidenceVars = None):
        '''
        Constructor
        '''
        self.queryVars = queryVars
        self.evidenceVars = evidenceVars
        
    def getNonevidenceVars(self, bayesNet):
        nonevidenceVars = []
        
        for n in bayesNet.nodeList:
            if self.evidenceVars.has_key(n.var) == False:
                nonevidenceVars.append(n.var)
        
        return nonevidenceVars
    
    def getVal(self, varname):
        val = None
        if self.evidenceVars.has_key(varname):
            val = self.evidenceVars[varname]
        return val
    
    def getRandomInstance(self, bayesNet):
        nodeNum = len(bayesNet.nodeList)
        insRnd = np.random.uniform(0,1,nodeNum)
        
        instance = dict([])
        idx = 0
        for v in bayesNet.nodeList:
            if self.evidenceVars.has_key(v.var):
                instance[v.var] = self.evidenceVars[v.var]
            else:  
                # simulate bernoulli
                if insRnd[idx] >= 0.5:
                    val = "F"
                else:
                    val = "T"
                instance[v.var] = val    
            idx += 1
        
        return instance
         
    
    def queryVarsFiltering(self, samples):
        sampleNum = len(samples)
        
        for i in range(sampleNum):
            instance = samples[i]
            for v in instance.keys():
                if (v in self.queryVars) == False:
                    del instance[v]

        return samples
    
    def estProbByGibbsSample(self, bayesNet, sampleNum, sampleFrom = 0, plotHist = False):
        samples = self.doGibbsSample(bayesNet, sampleNum, sampleFrom)
        self.estimateProb(samples, plotHist)
        
    def doGibbsSample(self, bayesNet, sampleNum, sampleFrom = 0):
        
        noneveidenceVars = self.getNonevidenceVars(bayesNet)
        samples = []
        
        #generate initial sample
        instance = self.getRandomInstance(bayesNet)
        
        for t in range(sampleNum+sampleFrom):
            
            if t >= sampleFrom:
                samples.append(instance)
            
            instance = copy.deepcopy(instance)                    
        
            for nv in noneveidenceVars:
                # find markov blanket
                instance = self.gibbsUpdate(nv, bayesNet, instance)
        
        #samples = self.queryVarsFiltering(samples)
        
        return samples
        
    def getProb(self, node, instance):
        prob = 0.0
        pos = []
        pos.append(instance[node.var])
        if node.parents != None:
            for p in node.parents:
                pos.append(instance[p])
        pos = tuple(pos)

        prob = node[pos]
        return prob
        
    def gibbsUpdate(self, var, bayesNet, instance):
        
        # get Markov blanket
        markovBlanket = bayesNet.getMarkovBlanket(var)
        
        TVarInstance = copy.deepcopy(instance)
        TVarInstance[var] = 'T'
        FVarInstance = copy.deepcopy(instance)
        FVarInstance[var] = 'F'
        
        node = bayesNet.find(var)    
        
        TVal = np.log(self.getProb(node, TVarInstance))
        FVal = np.log(self.getProb(node, FVarInstance))
        
        for m in markovBlanket:
            node = bayesNet.find(m)
            TVal += np.log(self.getProb(node, TVarInstance))
            FVal += np.log(self.getProb(node, FVarInstance))
            
        TVal = np.exp(TVal)
        FVal = np.exp(FVal)
        
        ProbT = TVal / (TVal + FVal)
        
        if np.random.binomial(1, ProbT)==1:
            instance[var] = 'T'
        else:
            instance[var] = 'F'       
        
        return instance
    
    def convertToPos(self, instance):
        pos = []
        for p in self.queryVars:
            if instance[p] == "T":
                pos.append(0)
            elif instance[p] == "F":
                pos.append(1)
                
        pos = tuple(pos)        
        return pos            
    
    def estimateProb(self, samples, plotHist = False):
        shape = 2 * np.ones(len(self.queryVars), int)
        self.count = np.zeros(shape)
        self.hist = []
        idx = 0
        self.mixploting = []
        for s in samples:
            pos = self.convertToPos(s)
            self.mixploting.append(list(pos))
            self.count[pos] += 1
            idx += 1
            if idx % 10000 == 0:
                self.hist.append(self.count[0]/idx)
        
        #print self.count
        
        if plotHist == True:
            fig = plt.figure()
            ax = fig.add_subplot(111)
            It = np.arange(0, len(samples), 10000)
            ax.plot(It, self.hist, "ro-")
            ax.set_xlabel(r"iteration")
            ax.set_ylabel(r"estimated probability")
            
            #print self.mixploting
            
            fig2 = plt.figure()
            ax2 = fig2.add_subplot(111)
            It2 = np.arange(len(samples)-500, len(samples))
            ax2.plot(It2, self.mixploting[len(samples)-500:len(samples)], 'b.')
            ax2.set_xlabel(r"iteration")
            ax2.set_ylabel(r"sample value")
            plt.show()
                
        self.estProb = np.zeros(shape)
        self.estProb = self.count / (len(samples))        
        print self.estProb
        
    def printEstProb(self):
        Trans = "True", "False"
        Trans2 = {"T":"True", "F":"False"}
        text = ""
        condText = ""
        
        if len(self.evidenceVars) != 0:
            condText = "|";
            condLen = len(self.evidenceVars)
            idxCond = 0
            for k in self.evidenceVars.keys():                
                condText += "{}={}".format(k, Trans2[self.evidenceVars[k]])
                idxCond += 1
                if idxCond < condLen:
                    condText += ","
                    
        bitNum = len(self.queryVars) 
        for i in range(2**bitNum):
            pos = []
            nbr = i
            text += "P("            
            for j in range(bitNum):
                idx = int(nbr/(2**(bitNum-(j+1))))
                nbr -= idx * (2**(bitNum-(j+1)))
                
                text += "{}={}".format(self.queryVars[j], Trans[idx])
                if j < bitNum - 1:
                    text += ","
                    
                pos.append(idx)
            #print pos
            pos = tuple(pos) 
            #print pos          
            text += "{})={}\\n".format(condText, self.estProb[pos])
     
        return text           
                 
        