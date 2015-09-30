'''
Created on Jun 11, 2013

@author: joseph
'''

import pygraphviz as pgv
from Node import Node
from Node import NodeType
from Path import Path
from Path import Direction

class Network:
    
    def __init__(self, pgvNet, name):
        # A list of all of the nodes in the network.
        self.nodes = []
        self.name = name
        self.implicitEvidenceNodes = []
        for n in pgvNet.nodes():
            self.createNode(n)
        for e in pgvNet.edges():
            e0 = self.getNode(e[0])
            e1 = self.getNode(e[1])
            e0.children.append(e1)
            e1.parents.append(e0)
                    
    def preprocess(self):
        self.markNodesWithEvidenceDescendants()
        
        for n in self.nodes:
            if n.type == NodeType["Normal"]:
                noParentEvidence = True
                for p in n.parents:
                    if p.type == NodeType["Evidence"]:
                        noParentEvidence = False
                if noParentEvidence:
                    nn = self.createNode(n.name+"-I")
                    nn.type = NodeType["Evidence"]
                    self.link(nn, n)                
                    self.implicitEvidenceNodes.append(nn)
        
    def link(self, parentNode, childNode):
        parentNode.children.append(childNode)
        childNode.parents.append(parentNode)
    
    def isBlockingNode(self, prevName, currName, nextName):
        prevN = self.getNode(prevName)
        currN = self.getNode(currName)
        nextN = self.getNode(nextName)
        currDirect = Direction["Child"]
        nextDirect = Direction["Child"]
        for p in prevN.parents:
            if p == currN:
                currDirect = Direction["Parent"]
        for c in currN.parents:
            if c == nextN:
                nextDirect = Direction["Parent"]

        currentNode = (currN, currDirect)
        nextNode = (nextN, nextDirect)
        blocked = self.isBlocked(currentNode, nextNode)
        print "{}-{}-{}:{}".format(prevN,currN,nextN,blocked)
        return blocked
    
        
    def isBlocked(self, node, nextNode):
        """ Checks if "node" is blocked given the node before it.
        next: tuple(Node, Direction)
        node: tupe(Node, Direction)
        """

        if node[1] == nextNode[1]:
            # if they are in the same direction
            # prev->node->nextNode
            # prev<-node<-nextNode
            if node[0].type == NodeType["Evidence"]:
                return True
            else:
                return False
        elif nextNode[1] == Direction["Parent"] and node[1] == Direction["Child"]:
            # prev<-node->nextNode
            if node[0].type == NodeType["Evidence"]:
                return False
            else:
                if node[0].hasEvidenceDescendant == True:
                    return False
                else:
                    return True
        elif nextNode[1] == Direction["Child"] and node[1] == Direction["Parent"]:
            # prev->node<-nextNode
            if node[0].type == NodeType["Evidence"]:
                return True
            else:
                return False
                
        assert False
                            
    def createNode(self, name):
        n = Node(name)
        self.nodes.append(n)
        return n
    
    def markNodesPrunable(self):
        """ Prunes unneeded nodes."""
        
        for node in self.nodes:
            node.isPrunable = True
        
        # 1) Create a path for every query node. Each path will contain only that evidence node.
        startPaths = []
        for node in self.nodes:
            if node.type == NodeType["Query"]:
                p = Path(node, Direction["Start"])
                self.markActivePath(p)
                startPaths.append(p)
        
        #print "startPaths: {}".format(startPaths)
        # 2) Add the neighbors of each query node to the paths.
        paths = []
        for path in startPaths:
            for node in path[-1][0].parents:
                p = path.clone()
                p.append(node, Direction["Parent"])
                if node.type == NodeType["Evidence"]:
                    self.markActivePath(p)
                paths.append(p)
            for node in path[-1][0].children:
                p = path.clone()
                p.append(node, Direction["Child"])
                if node.type == NodeType["Evidence"]:
                    self.markActivePath(p)
                paths.append(p)
        
        # 3) Expand the paths as far as we can.
        while len(paths) > 0:
            #print "paths: {}".format(paths)
            prevPaths = paths
            paths = []
            for curPath in prevPaths:
                for node in curPath[-1][0].parents:
                    if not self.isBlocked(curPath[-1], (node, Direction["Parent"])) and not node in curPath:
                        p = curPath.clone()
                        p.append(node, Direction["Parent"])
                        paths.append(p)
                        if node.type == NodeType["Evidence"]:
                            self.markActivePath(p)
                for node in curPath[-1][0].children:
                    if not self.isBlocked(curPath[-1], (node, Direction["Child"])) and not node in curPath:
                        p = curPath.clone()
                        p.append(node, Direction["Child"])
                        paths.append(p)
                        if node.type == NodeType["Evidence"]:
                            self.markActivePath(p)
    
    def removeImplicitEvidenceNodes(self):
        for iNode in self.implicitEvidenceNodes:
            for node in self.nodes:
                if node == iNode:
                    self.nodes.remove(node)
                
    
    def isImplicitEvidenceNode(self, n):
        for i in self.implicitEvidenceNodes:
            if n==i:
                return True
        return False
    
    def markNodesWithEvidenceDescendants(self):
        """ Marks all nodes with observed descendants as such."""
        for n in self.nodes:
            if n.type == NodeType["Evidence"]:
                self.markParentsAsEvidence(n)
                    
    def markParentsAsEvidence(self, node):
        for p in node.parents:
            if p.type != NodeType["Evidence"] and p.hasEvidenceDescendant == False:
                p.hasEvidenceDescendant = True
                self.markParentsAsEvidence(p)                
    
    def getNode(self, nodeName):
        for n in self.nodes:
            if n.name == nodeName:
                return n
        return None
    
    def markActivePath(self, path):
        """ Marks all of the nodes in the given path as un-prunable."""
        for pair in path:
            pair[0].isPrunable = False

    def printPrunableNode(self):
        prunableSet = self.getPrunableNodes()
        print str(prunableSet)
        
    def getPrunableNodes(self):
        prunableSet = []
        for n in self.nodes:
            if n.isPrunable:
                prunableSet.append(n.name)
        prunableSet.sort()
        return prunableSet
    
    def plot(self, filename=None):
        net = pgv.AGraph(directed=True)
        for n in self.nodes:
            if n.isPrunable == True:
                net.add_node(n.name, style='dashed' )
            else:
                if n.type == NodeType["Query"]:
                    net.add_node(n.name, color='blue')
                elif n.type == NodeType["Evidence"]:
                    net.add_node(n.name, fillcolor='gray', style='filled')
                else:
                    net.add_node(n.name)

        for n in self.nodes:
            for c in n.children:
                net.add_edge(n.name, c.name)
                
        net.layout()
        if filename == None:
            filename = self.name+".png"
        net.draw(filename, format='png', prog='dot')
        