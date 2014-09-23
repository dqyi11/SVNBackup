'''
Created on Sep 23, 2014

@author: daqing_yi
'''

class RegionalAdjacencyList(object):

    def __init__(self):
        
        self.label = -1
        self.edgeStrength = 0.0
        self.edgePixelCount = 0
        self.next = None
        self.cur = None
        self.prev = None
        
    def insert(self, newRAL):
        
        if self.next == None:
            self.next = newRAL
            newRAL.next = None
            return 0
        
        exists = 0
        self.cur = self.next
        while self.cur!=None:
            if newRAL.label == self.cur.label:
                exists = 1
                break
            elif self.cur.next==None or self.cur.next.label > newRAL.label:
                newRAL.next = self.cur.next
                self.cur.next = newRAL
                break
            
            self.cur = self.cur.next
            
        return exists
        
    
            
        

        