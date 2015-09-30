'''
Created on Oct 3, 2013

@author: Joseph

    A        B        C
A  (0,0)   (-1,1)   (2,-2)
B  (2,-2)  (0,0)    (-1,1)  
C  (-1,1)  (1,-1)   (10,-10)
'''

import pulp;

def p2MinMax():
    prob1 = pulp.LpProblem("A, B C", pulp.LpMinimize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    p2_a = pulp.LpVariable("Player 2 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_b = pulp.LpVariable("Player 2 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_c = pulp.LpVariable("Player 2 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += 0.0*p2_a - 1.0*p2_b + 2.0*p2_c <= U1, "Constraint 1"
    prob1 += 2.0*p2_a + 0.0*p2_b - 1.0*p2_c <= U1, "Constraint 2"
    prob1 += -1.0*p2_a + 1.0*p2_b + 10.0*p2_c <= U1, "Constraint 3"
    prob1 += p2_a + p2_c + p2_b == 1.0, "Probability Constraint"
    
    prob1.writeLP("ABC_p2MinMax.lp");
    prob1.solve();
    
    print "MinMax results for player 2:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "a:", p2_a.varValue;
    print "b:", p2_b.varValue;
    print "c:", p2_c.varValue;

def p2MaxMin():
    prob1 = pulp.LpProblem("A, B C", pulp.LpMaximize);
    
    U2 = pulp.LpVariable("Utility 1", cat='Continuous');
    p2_a = pulp.LpVariable("Player 2 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_b = pulp.LpVariable("Player 2 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_c = pulp.LpVariable("Player 2 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U2, "Objective Function"
    prob1 += 0.0*p2_a + 2.0*p2_b - 2.0*p2_c >= U2, "Constraint 1"
    prob1 += -2.0*p2_a + 0.0*p2_b + 1.0*p2_c >= U2, "Constraint 2"
    prob1 += 1.0*p2_a - 1.0*p2_b - 10.0*p2_c >= U2, "Constraint 3"
    prob1 += p2_a + p2_c + p2_b == 1.0, "Probability Constraint"
    
    prob1.writeLP("ABC_p2MaxMin.lp");
    prob1.solve();
    
    print "MinMax results for player 2:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "a:", p2_a.varValue;
    print "b:", p2_b.varValue;
    print "c:", p2_c.varValue;
    
def p1MinMax():
    prob1 = pulp.LpProblem("A, B C", pulp.LpMinimize);
    
    U2 = pulp.LpVariable("Utility 2", cat='Continuous');
    p1_a = pulp.LpVariable("Player 1 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_b = pulp.LpVariable("Player 1 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_c = pulp.LpVariable("Player 1 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U2, "Objective Function"
    prob1 += 0.0*p1_a + 1.0*p1_b - 2.0*p1_c <= U2, "Constraint 1"
    prob1 += -2.0*p1_a + 0.0*p1_b + 1.0*p1_c <= U2, "Constraint 2"
    prob1 += 1.0*p1_a - 1.0*p1_b - 10.0*p1_c <= U2, "Constraint 3"
    prob1 += p1_a + p1_c + p1_b == 1.0, "Probability Constraint"
    
    prob1.writeLP("ABC_p1MinMax.lp");
    prob1.solve();
    
    print "MinMax results for player 1:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "a:", p1_a.varValue;
    print "b:", p1_b.varValue;
    print "c:", p1_c.varValue;

def p1MaxMin():
    prob1 = pulp.LpProblem("A, B C", pulp.LpMaximize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    p1_a = pulp.LpVariable("Player 2 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_b = pulp.LpVariable("Player 2 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_c = pulp.LpVariable("Player 2 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += 0.0*p1_a - 2.0*p1_b + 2.0*p1_c >= U1, "Constraint 1"
    prob1 += 2.0*p1_a + 0.0*p1_b - 1.0*p1_c >= U1, "Constraint 2"
    prob1 += -1.0*p1_a + 1.0*p1_b + 10.0*p1_c >= U1, "Constraint 3"
    prob1 += p1_a + p1_c + p1_b == 1.0, "Probability Constraint"
    
    prob1.writeLP("ABC_p2MaxMin.lp");
    prob1.solve();
    
    print "MaxMin results for player 1:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "a:", p1_a.varValue;
    print "b:", p1_b.varValue;
    print "c:", p1_c.varValue;



if __name__ == '__main__':
    p2MinMax()
    print
    p2MaxMin()
    print
    p1MinMax()
    print
    p1MaxMin()
    
    
    

    
    
    
