'''
Created on 2013-5-25

@author: Walter
'''

import numpy as np
import pygraphviz as pgv
import copy
import matplotlib
#matplotlib.use("WXAgg")
import matplotlib.pyplot as plt

from BayesNode import *        

class BayesNet(object):

    def __init__(self):
        '''
        Constructor
        '''
        self.nodeList = []
        self.childlistInited = False
        
    def addNode(self, node, color=None, fillColor=None):
        self.nodeList.append(node)
        if color != None:
            node.color = color
        if fillColor != None:
            node.fillColor = fillColor    
        
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
    
    def initChildrenList(self):
        for n in self.nodeList:
            for p in n.parents:
                self.find(p).children.append(n.var)
        self.childlistInited = True
    
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
        '''
        if self.childlistInited == True:
            n = self.find(nodeName)
            markovBlanket = list(set(n.parents).union(n.children))
        else:
            markovBlanket = list(set(self.getParents(nodeName)).union(set(self.getChildren(nodeName))))
        '''
        markovBlanket = self.getChildren(nodeName)
        return markovBlanket
    
    def init(self, refVal=True):
        for n in self.nodeList:
            if n.type == NodeType["DiscreteVar"]:
                n.initCPT(refVal)
    
    def plot(self, filename = None, plotformat = "png"):
        
        g = pgv.AGraph(directed=True)
        
        for n in self.nodeList:
            if n.color != None and n.fillColor != None:
                g.add_node(n.var, color=n.color, fillcolor=n.fillColor, style="filled")
            elif n.color != None:
                g.add_node(n.var, color=n.color)
            elif n.fillColor != None:
                g.add_node(n.var, fillcolor=n.fillColor, style="filled")
            else:
                g.add_node(n.var)
            #g.add_node(n.printCPT(), shape='box')
                    
        for n in self.nodeList:
            if n.parents != None:
                for p in n.parents:
                    g.add_edge(p, n.var)

        g.layout(prog='dot')
        outputFilename = 'file.png'
        if isinstance(filename, str):
            outputFilename = filename + '.' + plotformat
            
        g.draw(outputFilename, format=plotformat)
        
          
class Query(object):

    def __init__(self, queryVars, evidenceVars = None):
        '''
        Constructor
        '''
        self.queryVars = queryVars
        self.evidenceVars = evidenceVars
        self.refFunc = dict([])
        
    def addRefFunc(self, key, func):
        self.refFunc[key] = func
        
    def getNonevidenceVars(self, bayesNet):
        nonevidenceVars = []
        
        for n in bayesNet.nodeList:
            if self.evidenceVars.has_key(n.var) == False:
                nonevidenceVars.append(n.var)
        
        return nonevidenceVars

    def printQuery(self):
        text = "QUERY VAR: "
        for q in self.queryVars:
            text += q
            text += ", "
        text += "   EVIDENCE VAR: "
        for s in self.evidenceVars:
            text += " {} : {}, ".format(s, self.evidenceVars[s])
        return text
   
    def getVal(self, varname):
        val = None
        if self.evidenceVars.has_key(varname):
            val = self.evidenceVars[varname]
        return val
    
    def getRandomInstance(self, bayesNet):
        instance = dict([])        
        for v in bayesNet.nodeList:
            if self.evidenceVars.has_key(v.var):
                instance[v.var] = self.evidenceVars[v.var]
            else:  
                instance[v.var] = v.getRandomInitSample()
        
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
        
    def getInitInstance(self, bayesNet, initial):
        instance = dict([])
        
        for v in bayesNet.nodeList:
            if self.evidenceVars.has_key(v.var):
                instance[v.var] = self.evidenceVars[v.var]
            else:  
                instance[v.var] = initial[v.var]
        return instance
    
    def plotSamples(self, samples, plotRef = False):
        fig = []
        histFig = []
        sampleNum = len(samples)
        idx = 0
        for v in self.queryVars:
            data = []
            for i in range(sampleNum):
                inst = samples[i]
                data.append(inst[v])
            fig.append(plt.figure())
            ax = fig[idx].add_subplot(111)
            It = np.arange(0, sampleNum)
            ax.plot(It, data, "r-")
            ax.set_xlabel(r"iteration")
            ax.set_ylabel(r"value")
            ax.set_title(v)
            
            histFig.append(plt.figure())
            ax1 = histFig[idx].add_subplot(111)
            n, bins, patches = ax1.hist(data, 50, normed=1)
            if plotRef == True:
                bincenter = 0.5 * (bins[1:]+bins[:-1])
                ybin = self.refFunc[v](bincenter)
                ax1.plot(bincenter, ybin, 'r-')
            title = v + " (Mean: {})".format(np.mean(data))  
            print title
            ax1.set_title(title)
            
            idx += 1
            
        plt.show()
        
    def doGibbsSampleWithHM(self, bayesNet, initial, candsd, sampleNum, sampleFrom = 0, hFileWriter = None):
        
        nonevidenceVars = self.getNonevidenceVars(bayesNet)
        samples = []
        
        #generate initial sample
        instance = self.getInitInstance(bayesNet, initial)
        
        for t in range(sampleNum+sampleFrom):
            
            if t >= sampleFrom:
                samples.append(instance)
                if hFileWriter != None:
                    hFileWriter.write(str(instance)+"\n")
            
            instance = copy.deepcopy(instance)                    
        
            for nv in nonevidenceVars:
                instance[nv] = self.metropolisHastingUpdate(nv, bayesNet, instance, candsd)
        
        #samples = self.queryVarsFiltering(samples)        
        return samples
           
    def doGibbsSample(self, bayesNet, sampleNum, sampleFrom = 0):
        
        nonevidenceVars = self.getNonevidenceVars(bayesNet)
        samples = []
        
        #generate initial sample
        instance = self.getRandomInstance(bayesNet)
        
        for t in range(sampleNum+sampleFrom):
            
            if t >= sampleFrom:
                samples.append(instance)
            
            instance = copy.deepcopy(instance)                    
        
            for nv in nonevidenceVars:
                instance[nv] = self.gibbsUpdate(nv, bayesNet, instance)
        
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
    
    def metropolisHastingUpdate(self, var, bayesNet, instance, candsd):
        
        #get Markov blanket
        markovBlanket = bayesNet.getMarkovBlanket(var)
        
        node = bayesNet.find(var)
        
        if node.type == NodeType["Bernoulli"] or node.type == NodeType["DiscreteVar"]:
            
            T_prob = node.getProbability(instance)
            
            if np.random.uniform(0.0,1.0) < T_prob :
                return True
            else:
                return False            
            
        else:
            lastVal = instance[var]
            candVal =  np.random.normal(lastVal, candsd[var])
            
            if node.type == NodeType["Poisson"]:
                if candVal < 1:
                    return lastVal
                else:
                    candVal = np.around(candVal)
            elif node.type == NodeType["Gamma"]:
                if candVal < 0:
                    candVal = 0.0
            elif node.type == NodeType["InvGamma"]:
                if candVal <= 0:
                    candVal = 1e-20              
            elif node.type == NodeType["Beta"]:
                if candVal <= 0.0 or candVal > 1.0:
                    return lastVal                   
            
            candInstance = copy.deepcopy(instance)
            candInstance[var] = candVal
            
            log_lh_last = bayesNet.find(var).getLogLikelihood(lastVal, instance)
            log_lh_cand = bayesNet.find(var).getLogLikelihood(candVal, instance)
            
            for m in markovBlanket:
                log_lh_last += bayesNet.find(m).getLogLikelihood(instance[m],instance)
                log_lh_cand += bayesNet.find(m).getLogLikelihood(candInstance[m],candInstance)
                
            u = math.log(np.random.uniform(0.0,1.0))
            
            if u < log_lh_cand - log_lh_last:
                return candVal
            else:
                return lastVal
        
    def gibbsUpdate(self, var, bayesNet, instance):
        
        # get Markov blanket
        markovBlanket = bayesNet.getMarkovBlanket(var)
        
        TVarInstance = copy.deepcopy(instance)
        TVarInstance[var] = True
        FVarInstance = copy.deepcopy(instance)
        FVarInstance[var] = False
        
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
            return True
        else:
            return False      
    
    def convertToPos(self, instance):
        pos = []
        for p in self.queryVars:
            if instance[p] == True:
                pos.append(0)
            elif instance[p] == False:
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
    
    def analyzeSamples(self, samples):
        
        analyzeData = dict([])
        for s in samples:
            for v in self.queryVars:
                if (analyzeData.has_key(v) == False):
                    analyzeData[v] = [] 
                analyzeData[v].append(s[v])
                
        samplenum = len(samples) 
        ability = []
        for ad in analyzeData:
            analyzeData[ad].sort()            
            median = analyzeData[ad][samplenum//2]
            low = analyzeData[ad][int(.05 * samplenum)]
            high = analyzeData[ad][int(.95 * samplenum)]
            ability.append((ad, low, median, high))
            
        ability.sort(lambda x, y: cmp(x[2], y[2]))
        i = 1
        for ad, low, median, high, in ability:
            print '%d: %s %f; 90%% interval: (%f, %f)' % (i, ad, median, low, high)
            i += 1
        