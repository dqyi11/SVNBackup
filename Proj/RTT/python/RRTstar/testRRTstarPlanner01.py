'''
Created on Dec 14, 2014

@author: daqing_yi
'''
from RRTstarPlanner import *
from World import *

if __name__ == '__main__':
    
    ITERATION_NUM = 2000
    
    world = World(3)
    world.regionOperating.size[0] = 20.0
    world.regionOperating.size[1] = 20.0
    world.regionOperating.size[2] = 20.0
    
    world.regionGoal.center[0] = 2.0
    world.regionGoal.center[1] = 2.0
    world.regionGoal.center[2] = 2.0
    world.regionGoal.size[0] = 2.0
    world.regionGoal.size[0] = 2.0
    world.regionGoal.size[0] = 2.0 
    
    obs = Region(3)
    obs.center[0] = 0
    obs.center[1] = 0
    obs.center[2] = 6
    obs.size[0] = 10
    obs.size[1] = 10
    obs.size[2] = 8
    world.obstacles.append(obs)    
    
    planner = RRTstarPlanner(3)
    planner.world = world
    
    root = planner.getRootVertex()
    rootState = root.state
    rootState[0] = 0.0
    rootState[1] = 0.0
    rootState[2] = 0.0
    
    planner.initialize()
    
    # This parameter should be larger than 1.5 for asymptotic 
    # optimality. Larger values will weigh on optimization 
    # rather than exploration in the RRT* algorithm. Lower 
    # values, such as 0.1, should recover the RRT.
    planner.setGamma (1.5);
    
    for i in range(ITERATION_NUM):
        print "@Iter " + str(i)
        planner.iteration()
        
    traj, ret = planner.getBestTrajectory()
    print ret
    
    
    