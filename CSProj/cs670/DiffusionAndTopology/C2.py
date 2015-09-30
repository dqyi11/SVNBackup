from DiffusionModel import *

if __name__ == '__main__':
    model = NearestNeighborsDiffusionModel(30, 1, 1.0, 20)
    model.run(3.0, 0.01, 1.0)
    model.plotDynamics("C2")
    model.plotTrajectoryX("C2")
    model.plotFiedlerEigenVals("C2")
    model.plotAverageDegree("C2")

    
    print str(model.repulsionGraph.adjacencyMatrix)
    print str(model.attractionGraph.adjacencyMatrix)