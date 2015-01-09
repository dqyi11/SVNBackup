'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from kRRTstar import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './map.png'
    
    start = [40,40]
    goal = [500, 300]
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist 
    
    krrts = kRRTstar([640, 400], 10)
    krrts.loadMap(MAP_FILE)
    
    krrts_viz = RRTVisualizer(krrts)
    
    krrts.init(start, goal, calcDist)
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        krrts.extend()
        krrts_viz.update()