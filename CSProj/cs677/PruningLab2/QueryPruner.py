'''
Created on 2013-6-9

@author: Walter
'''

import copy
import pygraphviz as pgv
from Network import *
from Node import *
from EdgeGraph import *

class QueryPruner(object):

    def __init__(self, query, gvNet, name):
        self.name = name
        self.queryNodes = []
        self.evidenceNodes = []
        self.net = Network(gvNet, name)
               
        for q in query.queryVars:
            self.queryNodes.append(q)
            n = self.net.getNode(q)
            n.type = NodeType["Query"]
        for e in query.evidenceVars:
            self.evidenceNodes.append(e)
            n = self.net.getNode(e)
            n.type = NodeType["Evidence"]
            
        self.net.plot()
        
        #self.net.preprocess()

    def isEvidenceNode(self, node):
        isEvidence = False
        for e in self.evidenceNodes:
            if e == node:
                isEvidence = True
        return isEvidence
    
    def isQueryNode(self, node):
        isQuery = False
        for q in self.queryNodes:
            if q == node:
                isQuery = True
        return isQuery
    
    def draw(self):
        self.net.plot()
        
    def prune(self):
        self.net.plot(self.name+"-mod.png")
        self.net.markNodesPrunable()
        self.net.removeImplicitEvidenceNodes()
        self.net.printPrunableNode()
        self.net.plot(self.name+"-pruned.png")
        
    def prune2(self):
        edgeGraph = EdgeGraph(self.name, self.net)
        edgeGraph.plot()
    
            
class Query(object):
        
    def __init__(self, queryVars, evidenceVars = None):
        self.queryVars = queryVars
        self.evidenceVars = evidenceVars