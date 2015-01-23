'''
Created on Nov 17, 2014

@author: daqing_yi
'''

from TopologicalGraph import *

if __name__ == '__main__':
    
    g = TopologicalGraph()
    n1 = g.addNode('1')
    n2 = g.addNode('2')
    n3 = g.addNode('3')
    n4 = g.addNode('4')
    
    g.addEdge(n1, n2, 'e1')
    g.addEdge(n2, n3, 'e2')
    g.addEdge(n3, n4, 'e3')
    g.addEdge(n4, n1, 'e4')
    
    g.visualize('test_graph')