'''
Created on 2014-10-27

@author: Walter
'''

class UnionFindSet(object):

    def __init__(self):
        self.P = []
        self.currentlabel = 0
        
    def createLabel(self):
        label = self.currentlabel
        self.P.append(label)
        self.currentlabel += 1
        return label
    
    def setRoot(self, i, root):
        while self.P[i] < i:
            j = self.P[i]
            self.P[i] = root
            i = j
        self.P[i] = root
        
    def findRoot(self, i):
        while self.P[i] < i:
            i = self.P[i]
        return i
    
    def find(self, i):
        root = self.findRoot(i)
        self.setRoot(i, root)
        return root
    
    def union(self, i, j):
        if i != j:
            root = self.findRoot(i)
            rootj = self.findRoot(j)
            if root > rootj: root = rootj
            self.setRoot(j, root)
            self.setRoot(i, root)
    
    def flatten(self):
        for i in range(1, len(self.P)):
            self.P[i] = self.P[self.P[i]]
    
    def flattenL(self):
        k = 1
        for i in range(1, len(self.P)):
            if self.P[i] < i:
                self.P[i] = self.P[self.P[i]]
            else:
                self.P[i] = k
                k += 1
        
        