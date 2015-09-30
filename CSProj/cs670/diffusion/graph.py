'''
Created on Nov 8, 2013

@author: daqing_yi
'''

import numpy as np;
import pygraphviz as pgv;

class TopoGraph(object):
    '''
    classdocs
    '''

    def __init__(self, nodeNum):
        '''
        Constructor
        '''
        self.nodeNum = nodeNum;
        self.nodes = np.arange(nodeNum);
        self.adjacency = np.zeros((nodeNum, nodeNum), int);
        #for i in range(nodeNum):
        #    self.adjacency[i,i] = 1;
        
    def connect(self, i, j):
        
        self.adjacency[i,j] = 1;
        self.adjacency[j,i] = 1;
             
    def getDMatrix(self):
        DMatrix = np.zeros((self.nodeNum, self.nodeNum), int);
        
        for i in range(self.nodeNum):
            DMatrix[i,i] = np.sum(self.adjacency[i,:]);
        
        return DMatrix;
    
    def getAMatrix(self):
        AMatrix = np.zeros((self.nodeNum, self.nodeNum), int);
        
        AMatrix = np.copy(self.adjacency);
        
        return AMatrix;
    
    def getLMatrix(self):
        
        D = self.getDMatrix();
        A = self.getAMatrix();
        
        L = D - A;
        return L;
    
    def getEigenValueOfL(self):
        
        eigVal, eigVec = np.linalg.eig(self.getLMatrix());
        eigVal = np.sort(eigVal);
        
        return eigVal, eigVec;
    
    def plot(self, filename):
        
        g = pgv.AGraph(directed=False)
        
        for i in range(self.nodeNum):
            g.add_node(str(i));
                    
        for i in range(self.nodeNum):
            for j in range(self.nodeNum):
                if self.adjacency[i,j] > 0:
                    g.add_edge(str(i), str(j));

        g.layout(prog='dot')
        outputFilename = 'file.png'
        if isinstance(filename, str):
            outputFilename = filename + '.png'
            
        g.draw(outputFilename)
        
    def dumpParam(self, filename):
        
        fileWriter = open(filename + "-Param" '.txt', 'w')
        
        eigVal, eigVec =  self.getEigenValueOfL();
        
        fileWriter.write("{}\n".format(eigVal));
        fileWriter.write("\n");
        fileWriter.write("{}\n".format(eigVec));
        fileWriter.write("\n");
        
        fileWriter.write("{}\n".format(self.getLMatrix()));
                         
        fileWriter.close();
            
        
class FullTopoGraph(TopoGraph):
    
    def __init__(self, nodeNum):        
        TopoGraph.__init__(self,nodeNum);
        self.adjacency = np.ones((nodeNum, nodeNum), int);
        
        for i in range(self.nodeNum):
            self.adjacency[i,i] = 0;
        
class RandomTopoGraph(TopoGraph):
    
    def __init__(self, nodeNum, prob=0.5):
        TopoGraph.__init__(self,nodeNum);
        self.randProb = prob;
        for i in range(nodeNum):
            for j in range(i, nodeNum):
                '''
                if i!=j:
                    self.adjacency[i,j] = np.random.binomial(1, self.randProb);
                '''
                val = np.random.binomial(1, self.randProb);
                self.adjacency[i,j] = val;
                self.adjacency[j,i] = val;
                    
                    
class RingTopoGraph(TopoGraph):
    
    def __init__(self, nodeNum):
        TopoGraph.__init__(self,nodeNum);
        for i in range(nodeNum-1):
            self.adjacency[i, i+1] = 1;
            self.adjacency[i+1, i] = 1;
        self.adjacency[0,nodeNum-1] = 1;
        self.adjacency[nodeNum-1,0] = 1;    
                            