'''
Created on 2013-11-14

@author: Walter
'''

'''
HIGH NOISE
'''

from DiffusionModel import *

if __name__ == '__main__':
    model = EuclideanDistanceDiffusionModel(30, 3.0, 1.0, 20)
    model.run(3.0, 0.01, 10.0)
    model.plotFiedlerEigenVals("C3")
    model.plotDynamics("C3")
    model.plotTrajectoryX("C3")
    model.plotAverageDegree("C3")
    
    #print np.linalg.eig(model.attractionGraph.getLaplacian())[0];
    #print np.linalg.eig(model.repulsionGraph.getLaplacian())[0];
    
    #print model.attractionHistEigenVal;
    #print model.repulsionHistEigenVal;