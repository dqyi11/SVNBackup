from DiffusionModel import *

if __name__ == '__main__':
    model = EuclideanDistanceDiffusionModel(30, 6.0, 1.0, 20)
    model.run(3.0, 0.01, 10.0)
    model.plotFiedlerEigenVals("C3-1")
    model.plotDynamics("C3-1")
    model.plotTrajectoryX("C3-1")
    model.plotAverageDegree("C3-1")