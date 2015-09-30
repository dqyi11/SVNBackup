'''
Created on Nov 11, 2013

@author: daqing_yi
'''

import numpy as np;

def calcFielderEigenvalue(laplacian):
    eigVal = np.linalg.eig(laplacian)[0];
    for i in range(len(eigVal)):
        if (np.iscomplexobj(eigVal[i])):
            eigVal[i] = np.abs(eigVal[i]);
        elif eigVal[i] < 0:
            eigVal[i] = 0.0;
    
    #eigVal = np.abs(eigVal);
    
    sortedEigVal = np.sort(eigVal);
    val = 0.00001;
    for i in range(len(sortedEigVal)):
        if sortedEigVal[i] > val:
            return sortedEigVal[i];
    return 0.0;    

class TopoGraph(object):
    '''
    classdocs
    '''


    def __init__(self, nodeNum):
        '''
        Constructor
        '''
        self.nodeNum = nodeNum;
        self.adjacencyMatrix = np.matrix(np.zeros(shape=(nodeNum, nodeNum)), np.int);
        
    def updateDegreeMatrix(self):
        self.degreeMatrix = np.matrix(np.zeros(shape=(self.nodeNum, self.nodeNum)), np.int);
        for i in range(self.nodeNum):
            self.degreeMatrix[i,i] = np.sum(self.adjacencyMatrix[i,:]);
        
    def getLaplacian(self):
        self.updateDegreeMatrix();
        L = self.degreeMatrix - self.adjacencyMatrix;
        assert (L == L.T).all()
        return L
    
    def getFiedlerEigenvalue(self):
        return calcFielderEigenvalue(self.getLaplacian());
    
        
    
    def getAverageDegree(self):
        val = np.sum(self.degreeMatrix) / np.double(self.nodeNum);
        return val;