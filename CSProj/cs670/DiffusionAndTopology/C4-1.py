from DiffusionModel import *

if __name__ == '__main__':
    model = NearestNeighborsDiffusionModel(30, 5, 1.0, 20)
    model.run(3.0, 0.01, 10.0)
    model.plotDynamics("C4-1")
    model.plotTrajectoryX("C4-1")
    model.plotFiedlerEigenVals("C4-1")
    model.plotAverageDegree("C4-1")

    
    print str(model.repulsionGraph.adjacencyMatrix)
    print str(model.attractionGraph.adjacencyMatrix)