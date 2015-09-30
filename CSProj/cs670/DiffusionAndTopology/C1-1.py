from DiffusionModel import *

if __name__ == '__main__':
    model = EuclideanDistanceDiffusionModel(30, 6.0, 1.0, 20)
    model.run(3.0, 0.01, 1.0)
    model.plotFiedlerEigenVals("C1-1")
    model.plotDynamics("C1-1")
    model.plotTrajectoryX("C1-1")
    model.plotAverageDegree("C1-1")
    
    #print np.linalg.eig(model.attractionGraph.getLaplacian())[0];
    #print np.linalg.eig(model.repulsionGraph.getLaplacian())[0];
    
    #print model.attractionHistEigenVal;
    #print model.repulsionHistEigenVal;