'''
Created on 2013-10-22

@author: Walter
'''

import numpy as np;

class Agent(object):
    '''
    classdocs
    '''

    def __init__(self, name):
        '''
        Constructor
        '''
        self.name = name;
        self.actionCnt = 0;
        self.actions = [];
        self.scores = [];
        self.totalScore = 0.0;
        
    def reset(self):
        self.actionCnt = 0;
        self.actions = [];
        self.scores = [];
        self.totalScore = 0.0;
        
    def action(self):
        self.actionCnt = self.actionCnt + 1;
        
    def setPayoffMatrix(self, matrix):
        self.payoffMatrix = matrix;
        
    def update(self):
        pass;
    
    def calcTotalScore(self):
        self.totalScore = 0.0;
        scoreLen = len(self.scores);
        for i in range(scoreLen):
            self.totalScore = self.totalScore + self.scores[i];
        
class AlwaysDefectAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "AwaysDefect";
        
    
    def action(self, compAction):
        Agent.action(self);
        return "D";
        
        
class RandomAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "Random";
        self.prob = 0.5;
        
    def setProbability(self, prob):
        self.prob = prob;
        
    def action(self, compAction):
        Agent.action(self);
        
        val = np.random.random();
        if val <= self.prob:
            return "C";
        
        return "D";

        
class AlwaysCooperateAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "AlwaysCooperate";
        
    def action(self, compAction):
        Agent.action(self);
        return "C";


class TitForTatAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "TitForTat";
        
    '''
    In the tit-for-tat strategy,
    the agent begins by cooperating,
    and then plays whatever strategy the other agent played.
    ''' 
    def action(self, compAction):
        Agent.action(self);
        
        if self.actionCnt == 1:
            return "C";
        
        return compAction;
        

class TitForTwoTatsAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "TitForTwoTats";
        self.compDefectCnt = 0;
        
    def reset(self):
        Agent.reset(self);
        self.compDefectCnt = 0;
        
    '''
    The tit-for-two-tats strategy is similar,
    except that P2 only defects if the other agent defects twice in a row, 
    but cooperates immediately after the other agent cooperates. 
    '''
    def action(self, compAction):
        Agent.action(self);
        if compAction == "C":
            self.compDefectCnt = 0;
        else:
            self.compDefectCnt = self.compDefectCnt + 1;
            
        if self.compDefectCnt < 2:
            return "C";
        else:
            return "D";
 
  
class PavlovAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "Pavlov";
        
    '''
    The Pavlov strategy begins by cooperating.  
    On subsequent turns, 
    whenever the two agents didn't agree on the previous last play,
    then the Pavlov agent defects;
    otherwise, the Pavlov agent cooperates.  
    '''     
    def action(self, compAction):
        Agent.action(self);
        if self.actionCnt == 1:
            self.lastAction = "C";
        else:
            if self.lastAction != compAction:
                self.lastAction = "D";
            else:
                self.lastAction = "C";
                
        return self.lastAction;
        
        
class WinStayLoseShiftAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "WinStayLoseShift";
        self.lastAction = "C";
        self.winned = True;
        self.maxScore = 0.0;
        self.secondMaxScore = 0.0;
        
    def reset(self):
        Agent.reset(self);
        self.lastAction = "C";
        self.winned = True;
        self.maxScore = 0.0;
        self.secondMaxScore = 0.0;
    
    '''
    The WinStay/LoseShift (WSLS)strategy begins by cooperating, 
    but then changes its behavior (C goes to D or D goes to C) whenever the agent doesn't win.
    Winning occurs whenever the agent gets either its most preferred or next most preferred result.  
    '''
    def action(self, compAction):
        Agent.action(self);
        if self.winned == False:
            if self.lastAction == "C":
                self.lastAction = "D";
            else:
                self.lastAction = "C";
                
        return self.lastAction;
    
    def setPayoffMatrix(self, matrix):
        Agent.setPayoffMatrix(self, matrix);
        scores = [];
        scores.append(matrix[0][0]);
        scores.append(matrix[0][1]);
        scores.append(matrix[1][0]);
        scores.append(matrix[1][1]);
        scores.sort();
        self.maxScore = scores[3];
        self.secondMaxScore = scores[2];        
    
    def setWinned(self, winned):
        self.winned = winned;   
        
    def update(self):
        '''
        check if agent winned
        '''
        roundNum = len(self.scores);
        score = self.scores[roundNum-1];
        if self.maxScore > score and self.secondMaxScore > score:
            self.setWinned(False);
        else:
            self.setWinned(True);
        
class NeverForgiveAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "NeverForgive";
        self.betrayed = False;
        
    def reset(self):
        Agent.reset(self);
        self.betrayed = False;
        
    def action(self, compAction):
        Agent.action(self);
        if self.actionCnt == 1:
            return "C";
        else:
            if compAction == "D":
                self.betrayed = True;
            if self.betrayed == True:
                return "D";
            else:
                return "C";
                
class GuessTwoStepsAgent(Agent):
    
    def __init__(self, name):
        Agent.__init__(self, name);
        self.type = "GuessTwoSteps";
        self.state = "UNKNOWN";
        self.guess = "UNKNOWN";
        
    def reset(self):
        Agent.reset(self);
        self.state = "UNKNOWN";
        self.guess = "UNKNOWN";
        
    def action(self, compAction):
        Agent.action(self);
        myAction = "D";
        if self.actionCnt == 2:
            if compAction == "D":
                #D
                self.guess = "AD_RND";
            elif compAction == "C":
                #C
                self.guess = "AC_NF_TFT_TF2T";
        elif self.actionCnt == 3:
            if self.guess == "AD_RND":
                # DD or DC
                self.state = "PLAY_AD";
                              
            elif self.guess == "AC_NF_TFT_TF2T":
                if compAction == "C":
                    #CC
                    self.guess = "AC_TF2T";
                    myAction = "D";
                elif compAction == "D":
                    #CD
                    self.guess = "TFT_NF";
                    myAction = "C";
                    self.state = "PLAY_TFT";
        
        elif self.actionCnt == 4:
            if self.state == "UNKNOWN":
                if compAction == "C":
                    #CCC 
                    self.state = "PLAY_AD";
                elif compAction == "D":
                    #CCD
                    self.state = "PLAY_TFT";

        if self.state == "PLAY_TFT":
            if self.actionCnt <= 4:
                myAction = "C";
            else:
                myAction = compAction;
        else:
            myAction = "D";
            
        return myAction;