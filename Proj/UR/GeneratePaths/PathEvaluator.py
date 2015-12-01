'''
Created on Nov 25, 2015

@author: walter
'''

from Path import *
import numpy as np

def evaluatePathDistance( path, sample_num = 100 ):
    
    straight_path = Path()
    straight_path.waypoints.append( path.waypoints[0] )
    straight_path.waypoints.append( path.waypoints[len(path.waypoints)-1])

    dist = 0.0
    sample_path = path.resample(sample_num)
    sample_straight_path = straight_path.resample(sample_num)
    for i in range(sample_num):
        dist += np.sqrt( (sample_path[i][0] - sample_straight_path[i][0])**2 + (sample_path[i][1] - sample_straight_path[i][1])**2 )
    return dist
        
        
    