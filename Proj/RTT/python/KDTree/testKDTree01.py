'''
Created on Dec 30, 2014

@author: daqing_yi
'''

from kdtree import *

if __name__ == '__main__':
    
    root = createKDTree([(2,3)], 2, ref_list=['A'])
    
    for x in root.inorder():
        print x.data, x.ref
        
    print "---"
    
    root.add((5,4),'B')
    
    for x in root.inorder():
        print x.data, x.ref
        
    print "---"
    
    
    root.add((9,6),'C')
    root.add((4,7),'D')
    root.add((8,1),'E')
    root.add((7,2),'F')
    
    print 'Finding nearest to (9,6) '
    results = root.search_nn((9,6))
    print results[0].data, results[0].ref, results[1]
    
    print "---"
    
    print 'Finding 2 near to (9,2) '
    results =  root.search_knn((9,2), 2)
    for node, dist in results:
        print node.data, node.ref, dist
        
    print "---"
        
    print "Finding distance < 8 to (9,2) "
    results = root.search_nn_dist((9,2), 8)
    for node in results:
        print node.data, node.ref
        
    print "---"
    
    print "Finding nearest to (9,2) "
    results = root.search_nn((9,2))
    print results[0].data, results[0].ref, results[1]
    
    '''
    print "Removing (8,1)"
    root.remove((8,1))
        
    print "---"
    
    results = root.search_nn((9,2))
    print results[0].data, results[0].ref, results[1]
    '''
    
    print "---"
    
    for x in root.inorder():
        print x.data, x.ref
    
    
        