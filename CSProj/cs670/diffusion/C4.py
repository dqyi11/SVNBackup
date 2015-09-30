'''
Created on Nov 8, 2013

@author: daqing_yi
'''

# Random graph with 10 nodes

from graph import *;
from DiffusionModel import *;

if __name__ == '__main__':
    
    A = RandomTopoGraph(10);
    eigVal, eigVec =  A.getEigenValueOfL();
    '''
    print A.getLMatrix();
    print eigVal;
    print eigVec;
    '''
    
    name = "C4";
    A.plot(name);
    A.dumpParam(name);
     
    model = DiffusionModel(10, A.getLMatrix());
      
    model.run(5);
    
    model.plotDynamics(name);
    model.plotVelocityX(name);
    model.plotVelocityY(name);
    model.plotTrajectoryX(name);
    model.plotTrajectoryY(name);
    
    model.dumpPosX(name);
    model.dumpPosY(name);
    model.dumpVelX(name);
    model.dumpVelY(name);
    
    