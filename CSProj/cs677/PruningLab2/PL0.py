'''
Created on 2013-6-9

@author: Walter
'''

import pygraphviz as pgv
from QueryPruner import *

if __name__ == '__main__':
    
    Net1 = pgv.AGraph("net0-1.dot")
    
    q1_1_name = "Net0-1-1"
    q1_1 = Query(["C"], {"A":True})
    q1_1p = QueryPruner(q1_1, Net1, q1_1_name)
    q1_1p.prune2()
    
    if [] == q1_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q1_1_name)
    else:
        print "{}: FAIL!".format(q1_1_name)

    q1_2_name = "Net0-1-2"
    q1_2 = Query(["A"], {"C":True})
    q1_2p = QueryPruner(q1_2, Net1, q1_2_name)
    q1_2p.prune2()
    if [] == q1_2p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q1_2_name)
    else:
        print "{}: FAIL!".format(q1_2_name)        

    q1_3_name = "Net0-1-3"
    q1_3 = Query(["C"], {"B":True})
    q1_3p = QueryPruner(q1_3, Net1, q1_3_name)
    q1_3p.prune2()
    if ["A"] == q1_3p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q1_3_name)
    else:
        print "{}: FAIL!".format(q1_3_name)     
    
    
    Net2 = pgv.AGraph("net0-2.dot")
    
    q2_1_name = "Net0-2-1"
    q2_1 = Query(["A"], {"C":True})
    q2_1p = QueryPruner(q2_1, Net2, "Net0-2-1")
    q2_1p.prune2()
    if ["B","C"] == q2_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q2_1_name)
    else:
        print "{}: FAIL!".format(q2_1_name) 
    
    q2_2_name = "Net0-2-2"
    q2_2 = Query(["A"], {"B":True})
    q2_2p = QueryPruner(q2_2, Net2, q2_2_name)
    q2_2p.prune2()    
    if [] == q2_2p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q2_2_name)
    else:
        print "{}: FAIL!".format(q2_2_name) 
    
    Net3 = pgv.AGraph("net0-3.dot")
    
    q3_1_name = "Net0-3-1"
    q3_1 = Query(["C"], {"B":True})
    q3_1p = QueryPruner(q3_1, Net3, q3_1_name)
    q3_1p.prune2()  
    if [] == q3_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q3_1_name)
    else:
        print "{}: FAIL!".format(q3_1_name)     
    
    q3_2_name = "Net0-3-2"
    q3_2 = Query(["C"], {"A":True})
    q3_2p = QueryPruner(q3_2, Net3, q3_2_name)
    q3_2p.prune2()
    if ["B"] == q3_2p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q3_2_name)
    else:
        print "{}: FAIL!".format(q3_2_name)
    
    Net4 = pgv.AGraph("net0-4.dot")
    
    q4_1_name = "Net0-4-1"
    q4_1 = Query(["A"], {"D":True})
    q4_1p = QueryPruner(q4_1, Net4, q4_1_name)
    q4_1p.prune2()
    if [] == q4_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q4_1_name)
    else:
        print "{}: FAIL!".format(q4_1_name)
    
    q5_1_name = "Net0-5-1"
    Net5 = pgv.AGraph("net0-5.dot")
    q5_1 = Query(["A"],{})
    q5_1p = QueryPruner(q5_1, Net5, q5_1_name)
    q5_1p.prune2()
    if [] == q5_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q5_1_name)
    else:
        print "{}: FAIL!".format(q5_1_name)
    
    q5_2_name = "Net0-5-2"
    q5_2 = Query(["A"],{"C":True})
    q5_2p = QueryPruner(q5_2, Net5, q5_2_name)
    q5_2p.prune2()
    if [] == q5_2p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q5_2_name)
    else:
        print "{}: FAIL!".format(q5_2_name)
    
    Net6 = pgv.AGraph("net0-6.dot")
    q6_1_name = "Net0-6-1"
    q6_1 = Query(["A"],{"I":True})
    q6_1p = QueryPruner(q6_1, Net6, q6_1_name)
    q6_1p.prune2()
    if [] == q6_1p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q6_1_name)
    else:
        print "{}: FAIL!".format(q6_1_name) 
        
    q6_2_name = "Net0-6-2"
    q6_2 = Query(["A"],{"H":True})
    q6_2p = QueryPruner(q6_2, Net6, q6_2_name)
    q6_2p.prune2()
    if [] == q6_2p.net.getPrunableNodes():
        print "{}: SUCCESS!".format(q6_2_name)
    else:
        print "{}: FAIL!".format(q6_2_name)