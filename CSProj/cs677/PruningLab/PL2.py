'''
Created on 2013-6-12

@author: Walter
'''

import pygraphviz as pgv
from QueryPruner import *

if __name__ == '__main__':
    
    Net1 = pgv.AGraph("net2-1.dot")
    
    q1_1_name = "Net2-1-1"
    q1_1 = Query(["Q1","Q2"], {"C":True,"I":True})
    q1_1p = QueryPruner(q1_1, Net1, q1_1_name)
    q1_1p.prune()
    if ["B","D","F"] == q1_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q1_1_name)
    else:
        print "{}: FAIL!".format(q1_1_name) 
    
    q1_2_name = "Net2-1-2"
    q1_2 = Query(["Q1","Q2"], {"E":True,"G":True})
    q1_2p = QueryPruner(q1_2, Net1, q1_2_name)
    q1_2p.prune()
    if ["A","B","C","D","E"] == q1_2p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q1_2_name)
    else:
        print "{}: FAIL!".format(q1_2_name)
        
    