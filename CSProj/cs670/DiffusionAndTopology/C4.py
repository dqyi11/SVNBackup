from DiffusionModel import *

if __name__ == '__main__':
    model = NearestNeighborsDiffusionModel(30, 1, 1.0, 20)
    model.run(3.0, 0.01, 10.0)
    model.plotDynamics("C4")
    model.plotTrajectoryX("C4")
    model.plotFiedlerEigenVals("C4")
    model.plotAverageDegree("C4")

    
    print str(model.repulsionGraph.adjacencyMatrix)
    print str(model.attractionGraph.adjacencyMatrix)