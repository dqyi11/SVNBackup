'''
Created on 2013-6-10

@author: Walter
'''
import pygraphviz as pgv
from QueryPruner import *

if __name__ == '__main__':
    
    Net1 = pgv.AGraph("net3-1.dot")
    
    q1_1_name = "Net3-1-1"
    q1_1 = Query(["Q1","Q2"], {"H":True})
    q1_1p = QueryPruner(q1_1, Net1, q1_1_name)
    q1_1p.prune()
    '''
    if ["B","D","F"] == q1_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q1_1_name)
    else:
        print "{}: FAIL!".format(q1_1_name) 
    '''
        
    Net2 = pgv.AGraph("net3-2.dot")
    
    q2_1_name = "Net3-2-1"
    q2_1 = Query(["Q1","Q2"], {"H":True, "I":True})
    q2_1p = QueryPruner(q2_1, Net2, q2_1_name)
    q2_1p.prune()
    
    Net3 = pgv.AGraph("net3-3.dot")
    
    q3_1_name = "Net3-3-1"
    q3_1 = Query(["O", "J"], {"H":True, "G":True})
    q3_1p = QueryPruner(q3_1, Net3, q3_1_name)
    q3_1p.prune()
    
    q3_2_name = "Net3-3-2"
    q3_2 = Query(["G", "N"], {"A":True, "M":True})
    q3_2p = QueryPruner(q3_2, Net3, q3_2_name)
    q3_2p.prune()
    
