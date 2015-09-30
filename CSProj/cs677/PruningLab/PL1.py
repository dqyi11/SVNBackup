'''
Created on 2013-6-12

@author: Walter
'''

import pygraphviz as pgv
from QueryPruner import *

if __name__ == '__main__':
    
    Net = pgv.AGraph("net1.dot")
    
    q1 = Query(["A"], {"I":True,"L":True})
    
    q1p = QueryPruner(q1, Net, "PL1")
    q1p.prune()
    
    q2 = Query(["B"], {"A":True})
    q2p = QueryPruner(q2, Net, "PL1-q2p")
    q2p.prune()