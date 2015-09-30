'''
Created on 2013-6-12

@author: Walter
'''

import pygraphviz as pgv
from QueryPruner import *
import numpy as np
import time
import random

if __name__ == '__main__':
    
    width = 10
    height = 10
    queryNum = 4
    evidenceNum = 10
    
    netConnectivity = 0.2
    
    name = time.strftime("%H%M%S")
    Net1 = pgv.AGraph(directed=True)
    nodeList = []
    for h in range(height):
        for w in range(width):
            nodeName = str(h)+","+str(w)
            Net1.add_node(nodeName)
            nodeList.append(nodeName)
            if h > 0:
                connectivity = np.random.random(width)
                for i in range(width):
                    if connectivity[i] < netConnectivity:
                        Net1.add_edge((str(h-1)+","+str(i)), nodeName)
        
    
    Net1.write(name+'.dot')        
    Net1.layout()
    Net1.draw(name+".png", prog="dot", format="png")
    
    queryList = []
    evidenceDict = {}
    queryIdx = np.random.uniform(0,width*height-1, queryNum)
    evidenceIdx = np.random.uniform(0, width*height-queryNum-1,evidenceNum)
    for qIdx in queryIdx:
        queryList.append(nodeList[np.int(qIdx)])
        nodeList.remove(nodeList[np.int(qIdx)])
    for eIdx in evidenceIdx:
        evidenceDict[nodeList[np.int(eIdx)]]=True
    
    q = Query(queryList, evidenceDict)
    qp = QueryPruner(q, Net1, name)
    sTicks = time.time()
    qp.prune()
    eTicks = time.time()
    
    print str(eTicks-sTicks)
    
    
    
            
            