'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kRRG import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist    
    
    krrg = kRRG([600, 400], 10)

    krrg_viz = RRTVisualizer(krrg)
    
    krrg.init([40,40], [320, 300], calcDist)
    
    for i in range(1000):
        #print i
        krrg.extend()
        krrg_viz.update()