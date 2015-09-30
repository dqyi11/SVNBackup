'''
Created on 2013-5-12

@author: Walter
'''


print("Hello world")

class Prob(object):

    def __init__(self, varname, freqs):
        '''
        Constructor
        '''
        self.varname = varname
        self.prob = {}
        self.values = []
        for (v, p) in freqs.items():
                self[v] = p
                
    def __getitem__(self, val):
        return self.prob[val]
    
    def __setitem__(self, val, p):
        self.prob[val] = p

P1 = Prob('X', {'T':0.2, 'F':0.8})
print(P1['T'])        
        
class JointProb(object):
    
    def __init__(self, variables):
        self.variables = variables
        self.prob = {}
        self.vals = []
        
    def __getitem__ (self, val):
        return self.prob[val]
    
    def __setitem__(self, vals, p):
         if vals not in self.vals:
            self.vals.append(vals)
         self.prob[vals] = p
        
P2 = JointProb(['X','Y']);
P2[1,1] = 0.3

print(P2[1,1])
        
        