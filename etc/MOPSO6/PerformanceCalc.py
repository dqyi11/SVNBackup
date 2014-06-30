'''
Created on Jan 22, 2014

@author: daqing_yi
'''

import numpy as np;

def calcDiversity(swarm):
    
    distanceList = [];
    if len(swarm) <= 1:
        return 0.0;
    
    for a in swarm:
        distance = [];
        for b in swarm:
            if a.index != b.index:
                dist = getDist(a.pos, b.pos, a.dimension);
                distance.append(dist);
    distanceList.append(np.amin(distance));   

    totalDiv = 0.0;
    for d in distanceList:
        totalDiv += d;
                
    return np.sqrt(totalDiv / len(distanceList));

def calcSpread(nondomParticles, swarm):
    
    distanceList = [];
    for a in nondomParticles:
        distance = [];
        for b in nondomParticles:
            if a.index != b.index:
                dist = swarm.calcFitAbsDelta(a.pos, b.pos);
                distance.append(dist);
        distanceList.append(np.amin(distance));
        
    totalSpd = 0.0;
    for d in distanceList:
        totalSpd += d;
        
    return np.sqrt(totalSpd / len(distanceList));
    
def calcHausdorffDist(swarm, pareto_set, precise):
    
    distance = [];
    for particle in swarm:
        dist = calcDist(particle, pareto_set, precise);
        distance.append(dist);
    return np.amax(distance);
    
def calcDist(particle, pareto_set, precise):
    
    distance = [];
    for ref in pareto_set:
        dist = getDist(particle.pos, ref.pos, particle.dimension);
        if dist <= precise:
            dist = 0.0;
        distance.append(dist);
    return np.amin(distance);

def getDist(pos1, pos2, dim):
    
    d = 0.0;
    for d in range(dim):
        d += (pos1[0,d] - pos2[0,d])**2;
    return np.sqrt(d);
                