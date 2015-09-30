'''
Created on Nov 11, 2013

@author: daqing_yi
'''

import numpy as np;
import matplotlib.pyplot as plt;
from TopoGraph import *;

class DiffusionModel(object):
    '''
    classdocs
    '''

    def __init__(self, agentNum, worldSize):
        '''
        Constructor
        '''
        self.agentNum = agentNum;
        self.agentPos = np.matrix((np.random.random((2, agentNum)) - 0.5) * worldSize);
        self.agentVel = np.matrix(np.zeros(shape=(2, agentNum)));
        self.repulsionGraph = TopoGraph(agentNum);
        self.attractionGraph = TopoGraph(agentNum);
        self.repulsionDegreeMatrixAverages = []
        self.attractionDegreeMatrixAverages = []
        
    def run(self, time, step=0.005, noiseAmp=1):
        
        self.agentHistPosX = [];
        self.agentHistPosY = [];
        self.agentHistVelX = [];
        self.agentHistVelY = [];
        self.repulsionHistEigenVal = [];
        self.attractionHistEigenVal = [];
        self.combinedHistEigenVal = [];
        self.step = step
        
        for i in range(self.agentNum):
            self.agentHistPosX.append([]);
            self.agentHistPosY.append([]);
            self.agentHistVelX.append([]);
            self.agentHistVelY.append([]);
        
        for t in np.arange(0, time, step):
            
            self.updateToplogy();
            repL = self.repulsionGraph.getLaplacian();
            attL = self.attractionGraph.getLaplacian();
            combined = attL - repL;
            
            noise = np.matrix(noiseAmp * (np.random.random((2,self.agentNum)) -0.5));
            
            temp = repL * self.agentPos.T - attL * self.agentPos.T + noise.T;
            self.agentVel = temp.T;
            self.agentPos = self.agentPos + self.agentVel * step;
            
            for i in range(self.agentNum):
                self.agentHistPosX[i].append(self.agentPos[0,i]);
                self.agentHistPosY[i].append(self.agentPos[1,i]);
                self.agentHistVelX[i].append(self.agentVel[0,i]);
                self.agentHistVelY[i].append(self.agentVel[1,i]);
            
            #print self.attractionGraph.getFiedlerEigenvalue()
            #print self.repulsionGraph.getFiedlerEigenvalue()
            
            self.attractionHistEigenVal.append(self.attractionGraph.getFiedlerEigenvalue());
            self.repulsionHistEigenVal.append(self.repulsionGraph.getFiedlerEigenvalue());
            self.combinedHistEigenVal.append(calcFielderEigenvalue(combined));
            
            diagonal = [self.repulsionGraph.degreeMatrix[r, r] for r in range(self.agentNum)]
            self.repulsionDegreeMatrixAverages.append(np.average(diagonal))
            diagonal = [self.attractionGraph.degreeMatrix[r, r] for r in range(self.agentNum)]
            self.attractionDegreeMatrixAverages.append(np.average(diagonal))
			
            #print self.attractionGraph.getLaplacian();
            #print self.repulsionGraph.getLaplacian();
            #print "\n";
            
            
            
    def updateToplogy(self):
        """ Override this in each child class."""
        raise NotImplementedError
    
    def plotDynamics(self, name = None):
        
        fig = plt.figure();
        ax = fig.add_subplot(111);
        '''
        for t in range(self.time + 1):
            #ax.scatter(self.nodePos[:, 0, t], self.nodePos[:, 1, t]);
            ax.plot(self.nodePos[:, 0, t], self.nodePos[:, 1, t]);
        '''
        for i in range(self.agentNum):
            ax.plot(self.agentHistPosX[i][:], self.agentHistPosY[i][:], '.-');
            
        ax.set_xlabel("X Position");
        ax.set_ylabel("Y Position");
        idx = [];
        for i in range(self.agentNum):
            idx.append("Node " + str(i));
        #ax.legend(idx);

        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_mo.png");
            
    def plotTrajectoryX(self, name = None):
        
        figX = plt.figure();
        axX = figX.add_subplot(111);
        
        for i in range(self.agentNum):
            ind = np.arange(len(self.agentHistPosX[i])) * self.step;
            axX.plot(ind, self.agentHistPosX[i][:]);
            
        idx = [];
        for i in range(self.agentNum):
            idx.append("Node " + str(i));
        #axX.legend(idx);
        
        axX.set_xlabel("Time");
        axX.set_ylabel("Position");
        axX.set_title("X");
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_posX.png");
            
    def plotTrajectoryY(self, name = None):
        
        figY = plt.figure();
        axY = figY.add_subplot(111);
        
        for i in range(self.agentNum):
            ind = np.arange(len(self.agentHistPosY[i])) * self.step;
            axY.plot(ind, self.agentHistPosY[i][:]);
            
        idx = [];
        for i in range(self.agentNum):
            idx.append("Node " + str(i));
        #axY.legend(idx);
        
        axY.set_xlabel("Time");
        axY.set_ylabel("Position");
        axY.set_title("Y");
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_posY.png");        
        
    def plotVelocityX(self, name=None):
        
        figX = plt.figure();
        axX = figX.add_subplot(111);
        
        for i in range(self.agentNum):
            ind = np.arange(len(self.agentHistVelX[i])) * self.step;
            axX.plot(ind, self.agentHistVelX[i]);
        
        idx = [];
        for i in range(self.agentNum):
            idx.append("Node " + str(i));
        #axX.legend(idx);
        
        axX.set_xlabel("Time");
        axX.set_ylabel("Position");
        axX.set_title("X Vel");
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_velX.png");
            

    def plotVelocityY(self, name=None):
        
        figY = plt.figure();
        axY = figY.add_subplot(111);
        
        for i in range(self.agentNum):
            ind = np.arange(len(self.agentHistVelY[i])) * self.step;
            axY.plot(ind, self.agentHistVelY[i]);
        
        idx = [];
        for i in range(self.agentNum):
            idx.append("Node " + str(i));
        #axY.legend(idx);

        axY.set_xlabel("Time");
        axY.set_ylabel("Position");
        axY.set_title("Y Vel");
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_velY.png");
            
    def plotFiedlerEigenVals(self, name=None):
        
        figX = plt.figure();
        axX = figX.add_subplot(111);
        
        
        ind = np.arange(len(self.repulsionHistEigenVal)) * self.step;
        axX.plot(ind, self.repulsionHistEigenVal);
        axX.plot(ind, self.attractionHistEigenVal);
        axX.plot(ind, self.combinedHistEigenVal);
    
        axX.legend(["Repulsion", "Attraction", "Combined"]);
        
        axX.set_xlabel("Time");
        axX.set_ylabel("Value");
        axX.set_title("Fiedler Eigenvalue");

        yMin, yMax = plt.ylim()
        yBuff = 0.1*(yMax - yMin)
        plt.ylim(yMin - yBuff, yMax + yBuff)
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_eigenvals.png");
    
    def plotAverageDegree(self, name=None):
        figX = plt.figure();
        axX = figX.add_subplot(111);
        
        
        ind = np.arange(len(self.repulsionDegreeMatrixAverages)) * self.step;
        axX.plot(ind, self.repulsionDegreeMatrixAverages);
        axX.plot(ind, self.attractionDegreeMatrixAverages);
    
        axX.legend(["Repulsion", "Attraction"],  bbox_to_anchor=(1.0, 0.2));
        
        axX.set_xlabel("Time");
        axX.set_ylabel("Value");
        axX.set_title("Average Degree");
 
        yMin, yMax = plt.ylim()
        yBuff = 0.1*(yMax - yMin)
        plt.ylim(yMin - yBuff, yMax + yBuff)
       
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_degree.png");
        
        
    def dumpPosX(self, filename):
        
        fileWriter = open(filename + "-PosX" '.txt', 'w')
        for i in range(self.agentNum):
            fileWriter.write("{}\n".format(self.agentHistPosX[i]));
        fileWriter.close();
        
    def dumpPosY(self, filename):
        
        fileWriter = open(filename + "-PosY" '.txt', 'w')
        for i in range(self.agentNum):
            fileWriter.write("{}\n".format(self.agentHistPosY[i]));
        fileWriter.close();
        
    def dumpVelX(self, filename):
        
        fileWriter = open(filename + "-VelX" '.txt', 'w')
        for i in range(self.agentNum):
            fileWriter.write("{}\n".format(self.agentHistVelX[i]));
        fileWriter.close();
        
    def dumpVelY(self, filename):
        
        fileWriter = open(filename + "-VelY" '.txt', 'w')
        for i in range(self.agentNum):
            fileWriter.write("{}\n".format(self.agentHistVelY[i]));
        fileWriter.close();
        
    def updateGraphByNearestNeighbors(self, graph, neighborNum):

        graph.adjacencyMatrix = np.matrix(np.zeros(graph.adjacencyMatrix.shape))
        for i in range(self.agentNum):
            distance = np.zeros(self.agentNum);
            for j in range(self.agentNum):
                if i == j:
                    distance[j] = 0.0;
                else:
                    distance[j] = (self.agentPos[0,i] - self.agentPos[0,j])**2 + (self.agentPos[1,i] - self.agentPos[1,j])**2;
            sortedIdx =  [sorted(distance).index(k)  for k in distance];

            for j in range(len(sortedIdx)):
                if j < neighborNum + 1 and j > 0:
                    graph.adjacencyMatrix[i, sortedIdx[j]] = 1;
                    graph.adjacencyMatrix[sortedIdx[j], i] = 1;

    
    def updateGraphByEuclideanDistance(self, graph, neighborDistance):
        """ An edge is created between 2 agents (nodes) iff they are within "range" of each other
            according to Euclidean distance."""
        graph.adjacencyMatrix = np.matrix(np.zeros(graph.adjacencyMatrix.shape))
        for a1 in range(self.agentNum):
            for a2 in range(a1+1, self.agentNum):
                if np.linalg.norm(self.agentPos[:,a1] - self.agentPos[:,a2]) <= neighborDistance:
                    graph.adjacencyMatrix[a1,a2] = 1
                    graph.adjacencyMatrix[a2,a1] = 1
                else:
                    graph.adjacencyMatrix[a1,a2] = 0
                    graph.adjacencyMatrix[a2,a1] = 0
        
        assert (graph.adjacencyMatrix == graph.adjacencyMatrix.T).all()
        
class EuclideanDistanceDiffusionModel(DiffusionModel):
    """Joseph does this."""
    def __init__(self, agentNum, attractionRange, repulsionRange, worldSize):
        DiffusionModel.__init__(self, agentNum, worldSize);
        self.attractionRange = attractionRange;
        self.repulsionRange = repulsionRange;
        
    
    def updateToplogy(self):
        self.updateGraphByEuclideanDistance(self.attractionGraph, self.attractionRange);
        self.updateGraphByEuclideanDistance(self.repulsionGraph, self.repulsionRange)
        
class NearestNeighborsDiffusionModel(DiffusionModel):
    
    def __init__(self, agentNum, attractionNeighborNum, repulsionRange, worldSize):
        DiffusionModel.__init__(self, agentNum, worldSize);
        self.attractionNeighborNum = attractionNeighborNum;
        self.repulsionRange = repulsionRange;
    
    def updateToplogy(self):
        self.updateGraphByNearestNeighbors(self.attractionGraph, self.attractionNeighborNum);
        self.updateGraphByEuclideanDistance(self.repulsionGraph, self.repulsionRange)
    
    
    
        