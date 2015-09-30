'''
Created on 2013-11-14

@author: Walter
'''
from DiffusionModel import *

if __name__ == '__main__':
    model = NearestNeighborsDiffusionModel(30, 5, 1.0, 20)
    model.run(3.0, 0.01, 1.0)
    model.plotDynamics("C2-1")
    model.plotTrajectoryX("C2-1")
    model.plotFiedlerEigenVals("C2-1")
    model.plotAverageDegree("C2-1")

    
    print str(model.repulsionGraph.adjacencyMatrix)
    print str(model.attractionGraph.adjacencyMatrix)