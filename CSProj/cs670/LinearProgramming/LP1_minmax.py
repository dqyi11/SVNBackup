'''
Created on 2013-10-1

@author: Walter
'''

'''
      LW     WL
LW  (2,1)  (0,0)
WL  (0,0)  (1,2)

'''


import pulp;

if __name__ == '__main__':
    
    
    # FOR HUSBAND STRATEGY
    
    prob1 = pulp.LpProblem("Battle of the Sexes 1", pulp.LpMinimize);

    
    U1 = pulp.LpVariable("Utility 1", cat='Continuous');
    h_lw = pulp.LpVariable("Husband LW", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    h_wl = pulp.LpVariable("Husband WL", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob1 += U1, "Objective Function"
    prob1 += 2.0 * h_lw <= U1, "Constraint 1"
    prob1 += 1.0 * h_wl <= U1, "Constraint 2"
    prob1 += h_lw + h_wl == 1.0, "Probability Constraint"
    
    prob1.writeLP("BattleOfTheSexes1.lp");
    prob1.solve();
    
    print "Status:", pulp.LpStatus[prob1.status];
    print "Husband:"
    print "LW:", h_lw.varValue;
    print "WL:", h_wl.varValue;
    print "Util 1:", U1.varValue;
    
    # FOR WIFE STRATEGY
    
    prob2 = pulp.LpProblem("Battle of the Sexes 2", pulp.LpMinimize);
    
    U2 = pulp.LpVariable("Utility 2", cat='Continuous');
    w_lw = pulp.LpVariable("Wife LW", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    w_wl = pulp.LpVariable("Wife WL", lowBound=0, upBound=1.0, cat='Continuous', e=None);
    
    prob2 += U2, "Objective Function"
    prob2 += 1.0 * w_lw <= U2, "Constraint 1"
    prob2 += 2.0 * w_wl <= U2, "Constraint 2"
    prob2 += w_lw + w_wl == 1.0, "Probability Constraint"
    
    prob2.writeLP("BattleOfTheSexes2.lp");
    prob2.solve();
    
    print "Status:", pulp.LpStatus[prob2.status];
    print "Wife:"
    print "LW:", w_lw.varValue;
    print "WL:", w_wl.varValue;
    print "Util 2:", U2.varValue;

    
    
    
    
    
    
    