'''
Created on 2013-10-1

@author: Walter
'''

'''
     F      S
F  (4,3)  (2,2)
S  (1,1)  (3,4)
'''


import pulp;

if __name__ == '__main__':
    
    
    # min max 
    prob1 = pulp.LpProblem("Battle of the Sexes 1", pulp.LpMinimize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    h_f = pulp.LpVariable("Husband F", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    h_s = pulp.LpVariable("Husband S", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += 4.0 * h_f + 2.0 * h_s <= U1, "Constraint 1"
    prob1 += 1.0 * h_f + 3.0 * h_s <= U1, "Constraint 2"
    prob1 += h_f + h_s == 1.0, "Probability Constraint"
    
    prob1.writeLP("BattleOfTheSexes1.lp");
    prob1.solve();
    
    print "Status:", pulp.LpStatus[prob1.status];
    print "Husband:"
    print "LW:", h_f.varValue;
    print "WL:", h_s.varValue;
    print "Util 1:", U1.varValue;
    
    # min max
    prob2 = pulp.LpProblem("Battle of the Sexes 2", pulp.LpMinimize);
    
    U2 = pulp.LpVariable("Utility 2", cat='Continuous');
    w_f = pulp.LpVariable("Wife F", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    w_s = pulp.LpVariable("Wife S", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob2 += U2, "Objective Function"
    prob2 += 3.0 * w_f + 1.0 * w_s <= U2, "Constraint 1"
    prob2 += 2.0 * w_f + 4.0 * w_s <= U2, "Constraint 2"
    prob2 += w_f + w_s == 1.0, "Probability Constraint"
    
    prob2.writeLP("BattleOfTheSexes2.lp");
    prob2.solve();
    
    print "Status:", pulp.LpStatus[prob1.status];
    print "Wife:"
    print "LW:", w_f.varValue;
    print "WL:", w_s.varValue;
    print "Util 2:", U2.varValue;
    
    # max min
    prob3 = pulp.LpProblem("Battle of the Sexes 2", pulp.LpMaximize);
    
    U2 = pulp.LpVariable("Utility 2", cat='Continuous');
    h_f = pulp.LpVariable("Wife F", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    h_s = pulp.LpVariable("Wife S", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob3 += U2, "Objective Function"
    prob3 += 3.0 * h_f + 2.0 * h_s >= U2, "Constraint 1"
    prob3 += 1.0 * h_f + 4.0 * h_s >= U2, "Constraint 2"
    prob3 += h_f + h_s == 1.0, "Probability Constraint"
    
    prob3.writeLP("BattleOfTheSexes3.lp");
    prob3.solve();
    
    print "Status:", pulp.LpStatus[prob3.status];
    print "Wife:"
    print "LW:", h_f.varValue;
    print "WL:", h_s.varValue;
    print "Util 2:", U2.varValue;
    
    # max min 
    prob4 = pulp.LpProblem("Battle of the Sexes 4", pulp.LpMaximize);
    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    w_f = pulp.LpVariable("Husband F", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    w_s = pulp.LpVariable("Husband S", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob4 += U1, "Objective Function"
    prob4 += 4.0 * w_f + 1.0 * w_s >= U1, "Constraint 1"
    prob4 += 2.0 * w_f + 3.0 * w_s >= U1, "Constraint 2"
    prob4 += w_f + w_s == 1.0, "Probability Constraint"
    
    prob4.writeLP("BattleOfTheSexes4.lp");
    prob4.solve();
    
    print "Status:", pulp.LpStatus[prob4.status];
    print "Husband:"
    print "LW:", w_f.varValue;
    print "WL:", w_s.varValue;
    print "Util 1:", U1.varValue;   