'''
Created on 2013-10-3

@author: Walter
'''

import pulp;
import numpy as np;

Util1 = np.array([[1, 2, 3],[0.5,4,5],[2.5,4.5,6.5]]);
Util2 = np.array([[-1, -2, -3],[-0.5,-4,-5],[-2.5,-4.5,-6.5]]);

#Util1 = [[8, 1, 1],[0,7,1],[0,0,9]];
#Util2 = [[-8, -1, -1],[0,-7,-1],[0,0,-9]];

'''
Util1 = np.array([[0.0, -1.0, 1.0],[1.0, 0.0,-1.0],[-1.0,1.0,0.0]]);
Util2 = np.array([[0.0, 1.0, -1.0],[-1.0, 0.0,1.0],[1.0,-1.0,0.0]]);
'''

'''
Util1 = np.array([[0, -1, 1],[1, 0,-1],[-1,1,0]]);
Util2 = np.array([[0, 1, -1],[-1, 0, 1],[1,-1,0]]);
'''


#print str(Util1[0,0]) + "," + str(Util1[0,1]) + "," + str(Util1[0,2])

print Util1
print Util2

def p2MinMax():
    prob1 = pulp.LpProblem("A, B, C", pulp.LpMinimize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    p2_a = pulp.LpVariable("Player 2 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_b = pulp.LpVariable("Player 2 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_c = pulp.LpVariable("Player 2 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += Util1[0,0] * p2_a + Util1[0,1] * p2_b + Util1[0,2] * p2_c, "Constraint 1"
    prob1 += Util1[1,0] * p2_a + Util1[1,1] * p2_b + Util1[1,2] * p2_c, "Constraint 2"
    prob1 += Util1[2,0] * p2_a + Util1[2,1] * p2_b + Util1[2,2] * p2_c, "Constraint 3"
    prob1 += p2_a + p2_b + p2_c == 1.0, "Probability Constraint"
    
    prob1.writeLP("abc_minmax.lp");
    prob1.solve();
    
    print "MinMax results for player 2:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "Util 1", U1.varValue;
    print "a:", p2_a.varValue;
    print "b:", p2_b.varValue;
    print "c:", p2_c.varValue;

def p2MaxMin():
    prob1 = pulp.LpProblem("A, B, C", pulp.LpMaximize);
    
    U2 = pulp.LpVariable("Utility 1", cat='Continuous');
    p2_a = pulp.LpVariable("Player 2 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_b = pulp.LpVariable("Player 2 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p2_c = pulp.LpVariable("Player 2 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U2, "Objective Function"
    prob1 += Util2[0,0] * p2_a + Util2[0,1] * p2_b + Util2[0,2] * p2_c, "Constraint 1"
    prob1 += Util2[1,0] * p2_a + Util2[1,1] * p2_b + Util2[1,2] * p2_c, "Constraint 2"
    prob1 += Util2[2,0] * p2_a + Util2[2,1] * p2_b + Util2[2,2] * p2_c, "Constraint 3"
    prob1 += p2_a + p2_b + p2_c == 1.0, "Probability Constraint"
    
    prob1.writeLP("abc_p2MaxMin.lp");
    prob1.solve();
    
    print "MinMax results for player 2:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "a:", p2_a.varValue;
    print "b:", p2_b.varValue;
    print "c:", p2_c.varValue;
    
def p1MinMax():
    prob1 = pulp.LpProblem("A, B, C", pulp.LpMinimize);
    
    U2 = pulp.LpVariable("Utility 2", cat='Continuous');
    p1_a = pulp.LpVariable("Player 1 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_b = pulp.LpVariable("Player 1 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_c = pulp.LpVariable("Player 1 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U2, "Objective Function"
    prob1 += Util2[0,0] * p1_a + Util2[0,1] * p1_b + Util2[0,2] * p1_c, "Constraint 1"
    prob1 += Util2[1,0] * p1_a + Util2[1,1] * p1_b + Util2[1,2] * p1_c, "Constraint 2"
    prob1 += Util2[2,0] * p1_a + Util2[2,1] * p1_b + Util2[2,2] * p1_c, "Constraint 3"
    prob1 += p1_a + p1_b + p1_c == 1.0, "Probability Constraint"
    
    prob1.writeLP("abc_p1MinMax.lp");
    prob1.solve();
    
    print "MinMax results for player 1:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "Util 2", U2.varValue;
    print "A:", p1_a.varValue;
    print "B:", p1_b.varValue;
    print "C:", p1_c.varValue;

def p1MaxMin():
    prob1 = pulp.LpProblem("A, B, C", pulp.LpMaximize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    p1_a = pulp.LpVariable("Player 1 A", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_b = pulp.LpVariable("Player 1 B", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    p1_c = pulp.LpVariable("Player 1 C", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += Util1[0,0] * p1_a + Util1[0,1] * p1_b + Util1[0,2] * p1_c, "Constraint 1"
    prob1 += Util1[1,0] * p1_a + Util1[1,1] * p1_b + Util1[1,2] * p1_c, "Constraint 2"
    prob1 += Util1[2,0] * p1_a + Util1[2,1] * p1_b + Util1[2,2] * p1_c, "Constraint 3"
    prob1 += p1_a + p1_b + p1_c == 1.0, "Probability Constraint"
    
    prob1.writeLP("abc_p1MaxMin.lp");
    prob1.solve();
    
    print "MaxMin results for player 1:"
    print "Status:", pulp.LpStatus[prob1.status];
    print "A:", p1_a.varValue;
    print "B:", p1_b.varValue;
    print "C:", p1_c.varValue;



if __name__ == '__main__':
    p2MinMax()
    print
    p2MaxMin()
    print
    p1MinMax()
    print
    p1MaxMin()
    
    
    
