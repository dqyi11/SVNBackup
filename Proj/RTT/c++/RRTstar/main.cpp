#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>


#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;



typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;




int main () {
    
    
    planner_t rrts;
    
    cout << "RRTstar is alive" << endl;
    
    // Create the dynamical system
    System system;
    
    // Three dimensional configuration space
    system.setNumDimensions (2);
    
    // Define the operating region
    system.regionOperating.setNumDimensions(2);
    system.regionOperating.center[0] = 0;
    system.regionOperating.center[1] = 0;
    system.regionOperating.size[0] = 600;
    system.regionOperating.size[1] = 400;
    
    // Define the goal region
    system.regionGoal.setNumDimensions(2);
    system.regionGoal.center[0] = 100;
    system.regionGoal.center[1] = 100;
    system.regionGoal.size[0] = 40;
    system.regionGoal.size[1] = 40;
    
    // Define the obstacle region
    region *obstacle;
    
    obstacle = new region;
    obstacle->setNumDimensions(2);
    obstacle->center[0] = 0;
    obstacle->center[1] = 200;
    obstacle->size[0] = 10;
    obstacle->size[1] = 10;
    
    system.obstacles.push_front (obstacle);  // Add the obstacle to the list
    
    // Add the system to the planner
    rrts.setSystem (system);
    
    // Set up the root vertex
    vertex_t &root = rrts.getRootVertex();
    State &rootState = root.getState();
    rootState[0] = 0;
    rootState[1] = 0;
    
    
    // Initialize the planner
    rrts.initialize ();
    
    // This parameter should be larger than 1.5 for asymptotic 
    //   optimality. Larger values will weigh on optimization 
    //   rather than exploration in the RRT* algorithm. Lower 
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (1.5);
    
    clock_t start = clock();
    
    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 10; i++)
    {
        rrts.iteration ();
        cout << "ITER " << i << " - NUM " << rrts.numVertices << endl;
    }
    
    clock_t finish = clock();
    cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;

    /*
    std::list<double*> best_trajectory;
    int ret = rrts.getBestTrajectory(best_trajectory);
    cout << "Best Trajectory " << ret << endl;
    */
    for ( std::list< Vertex<State,Trajectory,System>* >::iterator iter = rrts.listVertices.begin(); iter != rrts.listVertices.end(); iter++)
    {
        Vertex<State,Trajectory,System>* vex = (*iter);
        cout << vex->getState()[0] << " , " << vex->getState()[1] << endl;
    }

    return 1;
}

