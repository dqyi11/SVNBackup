'''
Created on Jan 3, 2015

@author: daqing_yi
'''
from kRRTstar import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    start = [40,40]
    goal = [320, 300]
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist    
    
    krrts = kRRTstar([600, 400], 10)

    krrts_viz = RRTVisualizer(krrts)
    
    krrts.init(start, goal, calcDist)
    
    for i in range(1000):
        #print i
        krrts.extend()
        krrts_viz.update()