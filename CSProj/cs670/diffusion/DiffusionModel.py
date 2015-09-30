'''
Created on Nov 8, 2013

@author: daqing_yi
'''

import numpy as np;
import matplotlib.pyplot as plt;
import matplotlib.animation as animation;

class DiffusionModel(object):
    '''
    classdocs
    '''

    def __init__(self, nodeNum, laplacian, initRegion=10.0, step=0.005):
        '''
        Constructor
        '''
        self.nodeNum = nodeNum;
        self.laplacian = laplacian;
        self.initRegion = initRegion;
        self.step = step;
        
    def run(self, time):
        
        self.time = time;
        self.nodePosX = [];
        self.nodePosY = [];
        self.nodeVelX = [];
        self.nodeVelY = [];
        
        for i in range(self.nodeNum):
            self.nodePosX.append([]);
            self.nodePosY.append([]);
            self.nodeVelX.append([]);
            self.nodeVelY.append([]);
        
        velocity = np.zeros((self.nodeNum,2));
        lastPos = np.zeros((self.nodeNum,2));
        
        lastPos[:,0] = np.multiply(np.random.random(self.nodeNum), self.initRegion);
        lastPos[:,1] = np.multiply(np.random.random(self.nodeNum), self.initRegion);
        
        for i in range(self.nodeNum):
            self.nodePosX[i].append(lastPos[i,0]);
            self.nodePosY[i].append(lastPos[i,1]);
        
        for t in np.arange(0, time+1, self.step):
            # for each node
            for i in range(self.nodeNum):
                
                # init vel to zero
                velocity[i,0] = 0;
                velocity[i,1] = 0;
                
                # calc vel
                for j in range(self.nodeNum):
                    velocity[i,0] = velocity[i,0] - self.laplacian[i,j] * lastPos[j,0] ;
                    velocity[i,1] = velocity[i,1] - self.laplacian[i,j] * lastPos[j,1] ;
                    
                lastPos[i,:] = lastPos[i,:] + velocity[i,:] * self.step;
                
                self.nodeVelX[i].append(velocity[i,0]);
                self.nodeVelY[i].append(velocity[i,1]);
                self.nodePosX[i].append(lastPos[i,0]);
                self.nodePosY[i].append(lastPos[i,1]);
                

    def plotDynamics(self, name = None):
        
        fig = plt.figure();
        ax = fig.add_subplot(111);
        '''
        for t in range(self.time + 1):
            #ax.scatter(self.nodePos[:, 0, t], self.nodePos[:, 1, t]);
            ax.plot(self.nodePos[:, 0, t], self.nodePos[:, 1, t]);
        '''
        for i in range(self.nodeNum):
            ax.plot(self.nodePosX[i][:], self.nodePosY[i][:], '.-');
            
        ax.set_xlabel("X Position");
        ax.set_ylabel("Y Position");
        idx = [];
        for i in range(self.nodeNum):
            idx.append("Node " + str(i));
        ax.legend(idx);

        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_mo.png");
            
    def plotTrajectoryX(self, name = None):
        
        figX = plt.figure();
        axX = figX.add_subplot(111);
        
        for i in range(self.nodeNum):
            ind = np.arange(len(self.nodePosX[i])) * self.step;
            axX.plot(ind, self.nodePosX[i][:]);
            
        idx = [];
        for i in range(self.nodeNum):
            idx.append("Node " + str(i));
        axX.legend(idx);
        
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
        
        for i in range(self.nodeNum):
            ind = np.arange(len(self.nodePosX[i])) * self.step;
            axY.plot(ind, self.nodePosY[i][:]);
            
        idx = [];
        for i in range(self.nodeNum):
            idx.append("Node " + str(i));
        axY.legend(idx);
        
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
        
        for i in range(self.nodeNum):
            ind = np.arange(len(self.nodeVelX[i])) * self.step;
            axX.plot(ind, self.nodeVelX[i]);
        
        idx = [];
        for i in range(self.nodeNum):
            idx.append("Node " + str(i));
        axX.legend(idx);
        
        axX.set_xlabel("Time");
        axX.set_ylabel("Velocity");
        axX.set_title("X Vel");
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_velX.png");
            
    def plotVelocityY(self, name=None):
        
        figY = plt.figure();
        axY = figY.add_subplot(111);
        
        for i in range(self.nodeNum):
            ind = np.arange(len(self.nodeVelX[i])) * self.step;
            axY.plot(ind, self.nodeVelY[i]);
        
        idx = [];
        for i in range(self.nodeNum):
            idx.append("Node " + str(i));
        axY.legend(idx);

        axY.set_xlabel("Time");
        axY.set_ylabel("Velocity");
        axY.set_title("Y Vel");
        
        if name == None:
            plt.show();
        else:
            plt.savefig(name+"_velY.png");
        
    def dumpPosX(self, filename):
        
        fileWriter = open(filename + "-PosX" '.txt', 'w')
        for i in range(self.nodeNum):
            for j in range(len(self.nodePosX[i])):
                fileWriter.write("{} ".format(self.nodePosX[i][j]));
            fileWriter.write("\n");
        fileWriter.close();
        
    def dumpPosY(self, filename):
        
        fileWriter = open(filename + "-PosY" '.txt', 'w')
        for i in range(self.nodeNum):
            for j in range(len(self.nodePosY[i])):
                fileWriter.write("{}, ".format(self.nodePosY[i][j]));
            fileWriter.write("\n");
        fileWriter.close();
        
    def dumpVelX(self, filename):
        
        fileWriter = open(filename + "-VelX" '.txt', 'w')
        for i in range(self.nodeNum):
            for j in range(len(self.nodeVelX[i])):
                fileWriter.write("{} ".format(self.nodeVelX[i][j]));
            fileWriter.write("\n");
        fileWriter.close();
        
    def dumpVelY(self, filename):
        
        fileWriter = open(filename + "-VelY" '.txt', 'w')
        for i in range(self.nodeNum):
            for j in range(len(self.nodeVelY[i])):
                fileWriter.write("{} ".format(self.nodeVelY[i][j]));
            fileWriter.write("\n");
        fileWriter.close();
          
                