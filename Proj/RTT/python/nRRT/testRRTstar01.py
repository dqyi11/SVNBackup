'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRTstar import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './lab-map-inferior.png'
    MAP_FILE = './lab-map-scaled.png'
    
    start = [40,40]
    goal = [320, 300]
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist   
    
    rrts = RRTstar([444, 989], 10)
    rrts.loadMap(MAP_FILE)
    
    rrts_viz = RRTVisualizer(rrts)
    
    rrts.init(start, goal, calcDist)
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        rrts.extend()
        rrts_viz.update()
    