'''
Created on Nov 8, 2013

@author: daqing_yi
'''

# Graph with 8 nodes and two connected components

from graph import *;
from DiffusionModel import *;

if __name__ == '__main__':

    A = TopoGraph(8);
    A.connect(1, 3);
    A.connect(0, 2);
    A.connect(2, 3);
    
    A.connect(4, 7);
    A.connect(4, 5);
    A.connect(5, 6);
    A.connect(6, 7);
    
    eigVal, eigVec =  A.getEigenValueOfL();
    
    '''
    print A.getLMatrix();
    print eigVal;
    print eigVec;
    '''
    
    name = "C5";
    A.plot(name);
    A.dumpParam(name);
     
    model = DiffusionModel(8, A.getLMatrix());
      
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
    