'''
Created on Nov 8, 2013

@author: daqing_yi
'''

# Ring graph with five nodes

from graph import *;
from DiffusionModel import *;

if __name__ == '__main__':
    
    A = RingTopoGraph(5);
    eigVal, eigVec =  A.getEigenValueOfL();
    
    '''
    print A.getLMatrix();
    print eigVal;
    print eigVec;
    '''
    
    name = "C3";
    A.plot(name);
    A.dumpParam(name);
     
    model = DiffusionModel(5, A.getLMatrix());
      
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
    