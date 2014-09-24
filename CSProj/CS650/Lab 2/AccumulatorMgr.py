'''
Created on Sep 23, 2014

@author: daqing_yi
'''

class Accumulator(object):
    
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        self.voters = []
        self.voters_weight = []
        
    def addVote(self, voter, weight):
        self.voters.append(voter)
        self.voters_weight.append(weight)

class AccumulatorMgr(object):

    def __init__(self):
        self.accumulators = []
        
    def findAccumulator(self, x, y, r):
        for a in self.accumulators:
            if a.x==x and a.y==y and a.r==r:
                return a
        return None
    
    def vote(self, x, y, r, voter, weight=1.0):
        
        accumulator = self.findAccumulator(x, y, r)
        if accumulator == None:
            accumulator = Accumulator(x, y, r)
        accumulator.addVote(voter, weight)
        
    
    