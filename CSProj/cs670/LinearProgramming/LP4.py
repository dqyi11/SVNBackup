'''
Created on 2013-10-3

@author: Walter
'''

'''
          Rock    Paper   Scissors
Rock     (1,3)    (0,0)   (0,0)
Paper    (0,0)    (2,2)   (0,0)
Scissors (0,0)    (0,0)   (3,1)
'''

import pulp;

def p2MinMax():
    prob1 = pulp.LpProblem("Rock, Paper Scissors", pulp.LpMinimize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    p2_rock = pulp.LpVariable("Player 2 Rock", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_paper = pulp.LpVariable("Player 2 Paper", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_scissors = pulp.LpVariable("Player 2 Scissors", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += p2_rock, "Constraint 1"
    prob1 += 2 * p2_paper, "Constraint 2"
    prob1 += 3 * p2_scissors <= U1, "Constraint 3"
    prob1 += p2_rock + p2_scissors + p2_paper == 1.0, "Probability Constraint"
    
    prob1.writeLP("NonZeroSum_RockPaperScissors_p2MinMax.lp");
    prob1.solve();
    
    print "MinMax results for player 2:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "rock:", p2_rock.varValue;
    print "paper:", p2_paper.varValue;
    print "scissors:", p2_scissors.varValue;

def p2MaxMin():
    prob1 = pulp.LpProblem("Rock, Paper Scissors", pulp.LpMaximize);
    
    U2 = pulp.LpVariable("Utility 1", cat='Continuous');
    p2_rock = pulp.LpVariable("Player 2 Rock", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_paper = pulp.LpVariable("Player 2 Paper", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_scissors = pulp.LpVariable("Player 2 Scissors", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U2, "Objective Function"
    prob1 += 3 * p2_rock >= U2, "Constraint 1"
    prob1 += 2 * p2_paper >= U2, "Constraint 2"
    prob1 += p2_scissors >= U2, "Constraint 3"
    prob1 += p2_rock + p2_scissors + p2_paper == 1.0, "Probability Constraint"
    
    prob1.writeLP("NonZeroSum_RockPaperScissors_p2MaxMin.lp");
    prob1.solve();
    
    print "MinMax results for player 2:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "rock:", p2_rock.varValue;
    print "paper:", p2_paper.varValue;
    print "scissors:", p2_scissors.varValue;
    
def p1MinMax():
    prob1 = pulp.LpProblem("Rock, Paper Scissors", pulp.LpMinimize);
    
    U2 = pulp.LpVariable("Utility 2", cat='Continuous');
    p1_rock = pulp.LpVariable("Player 1 Rock", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_paper = pulp.LpVariable("Player 1 Paper", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_scissors = pulp.LpVariable("Player 1 Scissors", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U2, "Objective Function"
    prob1 += 3 * p1_rock <= U2, "Constraint 1"
    prob1 += 2 * p1_paper <= U2, "Constraint 2"
    prob1 += p1_scissors <= U2, "Constraint 3"
    prob1 += p1_rock + p1_scissors + p1_paper == 1.0, "Probability Constraint"
    
    prob1.writeLP("NonZeroSum_RockPaperScissors_p1MinMax.lp");
    prob1.solve();
    
    print "MinMax results for player 1:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "rock:", p1_rock.varValue;
    print "paper:", p1_paper.varValue;
    print "scissors:", p1_scissors.varValue;

def p1MaxMin():
    prob1 = pulp.LpProblem("Rock, Paper Scissors", pulp.LpMaximize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    p1_rock = pulp.LpVariable("Player 2 Rock", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_paper = pulp.LpVariable("Player 2 Paper", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_scissors = pulp.LpVariable("Player 2 Scissors", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += p1_rock >= U1, "Constraint 1"
    prob1 += 2 * p1_paper >= U1, "Constraint 2"
    prob1 += 3 * p1_scissors >= U1, "Constraint 3"
    prob1 += p1_rock + p1_scissors + p1_paper == 1.0, "Probability Constraint"
    
    prob1.writeLP("NonZeroSum_RockPaperScissors_p2MaxMin.lp");
    prob1.solve();
    
    print "MaxMin results for player 1:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "rock:", p1_rock.varValue;
    print "paper:", p1_paper.varValue;
    print "scissors:", p1_scissors.varValue;



if __name__ == '__main__':
    p2MinMax()
    print
    p2MaxMin()
    print
    p1MinMax()
    print
    p1MaxMin()
    
    
    
