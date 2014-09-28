'''
Created on Sep 23, 2014

@author: daqing_yi
'''
import numpy as np

class Vote(object):
    
    def __init__(self, accumulator, voter, weight):
        self.accumulator = accumulator
        self.voter = voter
        self.weight = weight
        
class Voter(object):
    
    def __init__(self, pos_x, pos_y):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.votes = []
        self.totalWeights = 0.0

class Accumulator(object):
    
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        self.votes = []
        self.totalWeights = 0.0
        
class AccumulatorMgr(object):

    def __init__(self):
        self.accumulators = []
        
    def findAccumulator(self, x, y, r):
        for a in self.accumulators:
            if a.x==x and a.y==y and a.r==r:
                return a
        return None
    
class VoterMgr(object):
    
    def __init__(self):
        self.voters =[]
        
    def findVoter(self, pos_x, pos_y):
        for v in self.voters:
            if v.pos_x == pos_x and v.pos_y == pos_y:
                return v
        return None

        
class VoteGame(object):
    
    def __init__(self, width, height, radii):
        self.width = width
        self.height = height
        self.radii = radii
        
        self.accumulatorMgr = AccumulatorMgr()
        self.voterMgr = VoterMgr()
        self.votes = []
        
    def votesByUser(self, x, y, r, voter, weight=1):
        accumulator = self.accumulatorMgr.findAccumulator(x, y, r)
        if accumulator==None:
            accumulator = Accumulator(x, y, r)
            self.accumulatorMgr.accumulators.append(accumulator)
            
        vote = Vote(accumulator, voter, weight)
        voter.votes.append(vote)
        accumulator.votes.append(vote)
        self.votes.append(vote)
        
    def vote(self, x, y, r, voter_x, voter_y, weight=1):
        voter = self.voterMgr.findVoter(voter_x, voter_y)
        if voter==None:
            voter = Voter(voter_x, voter_y)
            self.voterMgr.voters.append(voter)
            
        accumulator = self.accumulatorMgr.findAccumulator(x, y, r)
        if accumulator==None:
            accumulator = Accumulator(x, y, r)
            self.accumulatorMgr.accumulators.append(accumulator)
            
        vote = Vote(accumulator, voter, weight)
        voter.votes.append(vote)
        accumulator.votes.append(vote)
        
    def updateWeightSum(self):
        for accumulator in self.accumulatorMgr.accumulators:
            accumulator.totalWeights = 0.0
            for v in accumulator.votes:
                accumulator.totalWeights += v.weight
                
        for voter in self.voterMgr.voters:
            voter.totalWeights = 0.0
            for v in voter.votes:
                voter.totalWeights += v.weight
        
    def weigthedRevote(self):
        
        for v in self.votes:
            if v.voter.totalWeights != 0.0:
                #print "before " + str(v.weight)
                v.weight = v.accumulator.totalWeights / float(v.voter.totalWeights)
                #print "after " + str(v.weight)
            else:
                v.weight = 0.0
                
        self.updateWeightSum()
        
        
        
    def dumpHoughImg(self):
        
        hough_img = np.zeros((self.width, self.height, len(self.radii)), np.float)
        
        for a in self.accumulatorMgr.accumulators:
            hough_img[a.x, a.y, a.r] = a.totalWeights
            
        return hough_img
    
    def dumpHoughImgByRadusIndex(self, r_idx):
        
        hough_img = np.zeros((self.width, self.height), np.float)
        
        for a in self.accumulatorMgr.accumulators:
            #print "accumulator " + str(a.r) + " " + str(a.x) + " " + str(a.y) + " " + str(a.totalWeights)
            if a.r == r_idx:
                hough_img[a.x, a.y] = a.totalWeights
        
        hough_img_min = np.min(hough_img.flatten())
        hough_img_max = np.max(hough_img.flatten())
    
        print "max " + str(hough_img_max)
        print "min " + str(hough_img_min)
        
        hough_img = (hough_img - hough_img_min) /(hough_img_max - hough_img_min)    
            
        return hough_img
        
        
