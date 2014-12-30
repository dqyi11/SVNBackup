'''
Created on Dec 30, 2014

@author: daqing_yi
'''

from kdtree import *

if __name__ == '__main__':
    
    root = create([(2,3)], 2)
    
    root.add((5,4))
    root.add((9,6))
    root.add((4,7))
    root.add((8,1))
    root.add((7,2))
    
    results =  root.search_knn((9,2), 2)
    for node, dist in results:
        print node.data, dist
        
    print "---"
        
    results = root.search_nn_dist((9,2), 8)
    for node in results:
        print node.data
        
    print "---"
    
    results = root.search_nn((9,2))
    print results[0].data, results[1]
        
    print "Removing (8,1)"
    root.remove((8,1))
        
    print "---"
    
    results = root.search_nn((9,2))
    print results[0].data, results[1]
    
    
        