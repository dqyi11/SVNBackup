'''
Created on Dec 14, 2014

@author: daqing_yi
'''
from RRTstarPlanner import *
from World import *
from RRTstarVisualizer import *

if __name__ == '__main__':
    
    ITERATION_NUM = 10
    
    world = World(2)
    world.regionOperating.center[0] = 0
    world.regionOperating.center[1] = 0
    world.regionOperating.size[0] = 600
    world.regionOperating.size[1] = 400
    
    world.regionGoal.center[0] = 100
    world.regionGoal.center[1] = 100
    world.regionGoal.size[0] = 40
    world.regionGoal.size[0] = 40

    
    obs = Region(2)
    obs.center[0] = 0
    obs.center[1] = 200
    obs.size[0] = 10
    obs.size[1] = 10
    world.obstacles.append(obs)    
    
    planner = RRTstarPlanner(2)
    planner.world = world
    
    root = planner.getRootVertex()
    rootState = root.state
    rootState[0] = 0
    rootState[1] = 0
    
    planner.initialize()
    
    # This parameter should be larger than 1.5 for asymptotic 
    # optimality. Larger values will weigh on optimization 
    # rather than exploration in the RRT* algorithm. Lower 
    # values, such as 0.1, should recover the RRT.
    planner.setGamma (1.5)
    
    viz = RRTstarVisualizer(planner.kdtree, [[-300,300],[-200,200]])
    
    for i in range(ITERATION_NUM):
        print "@Iter " + str(i) + " NUM " + str(planner.numVertices)
        planner.iteration()
        viz.update()
        
    traj, ret = planner.getBestTrajectory()
    print ret
    
    
    