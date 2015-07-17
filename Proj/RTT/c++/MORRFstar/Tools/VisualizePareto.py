'''
Created on Jul 17, 2015

@author: daqing_yi
'''

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

if __name__ == '__main__':
    
    filename = ""
    if len(sys.argv)>1:
        filename = sys.argv[1]
        
    scores = []    
    paths = []
    
    #filename = "paths.txt"
    
    axis_font = {'fontname':'Arial', 'size':'25'}
        
    with open(filename, 'r') as f1:
        data_lines = f1.readlines()
        scores_finished = False
        for data_line in data_lines:
            if data_line == "\n":
                scores_finished = True
            else:
                if scores_finished == False:
                    data_line = data_line.replace('\n', '')
                    score_strs = data_line.split("\t")
                    score = []
                    for score_str in score_strs:
                        if score_str != '':
                            score.append(float(score_str))
                    scores.append(score)                    
                else:
                    path = []
                    data_line = data_line.replace('\n', '')
                    points_str = data_line.split("\t")
                    for point_str in points_str:
                        if point_str != "":
                            pos_strs = point_str.split(" ")
                            pos = []
                            for pos_str in pos_strs:
                                pos.append(float(pos_str))
                            path.append(pos)
                    paths.append(path)
                    
    print "SCORE:"
    print scores
    print "PATHS:"
    print paths
    
    numObj = len(scores[0])
    numSolutions = len(scores)
    
    np_scores = np.zeros((numSolutions, numObj), np.double)
    for i in range(numSolutions):
        for k in range(numObj):
            np_scores[i,k] = scores[i][k]
    
    if numObj == 2:
        fig = plt.figure()     
        ax = fig.add_subplot(111)
        for i in range(numSolutions):
            if i < numObj:
                ax.plot(np_scores[i,0], np_scores[i,1],'bs')
            else:
                ax.plot(np_scores[i,0], np_scores[i,1],'ro')
                
        ax.set_xlabel("objective 1", **axis_font)
        ax.set_ylabel("objective 2", **axis_font)
        plt.show()
    elif numObj == 3:
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection='3d')
        
        # interpolation spline with scipy 
        from scipy import interpolate 
        
        tck0 = interpolate.bisplrep(np_scores[:,0], np_scores[:,1], np_scores[:,2]) 
        xmin = np.min(np_scores[:,0])
        xmax = np.max(np_scores[:,0])
        ymin = np.min(np_scores[:,1])
        ymax = np.max(np_scores[:,1])
        xnew,ynew = np.mgrid[xmin:xmax:70j,ymin:ymax:70j] 
        znew = interpolate.bisplev(xnew[:,0],ynew[0,:],tck0) 
        ax1.plot_surface(xnew, ynew, znew, linewidth=0.0, color='y', alpha = 0.25)
        
        ax1.scatter(np_scores[0,0], np_scores[1,1], np_scores[2,2], c='g', marker='p')
        for i in range(numSolutions):
            if i < numObj:
                ax1.scatter(np_scores[i,0], np_scores[i,1], np_scores[i,2], c ='b', marker='s')
            else:
                ax1.scatter(np_scores[i,0], np_scores[i,1], np_scores[i,2], c = 'r', marker='o')
                
        ax1.set_xlabel('Objective 1', **axis_font)
        ax1.set_ylabel('Objective 2', **axis_font)
        ax1.set_zlabel('Objective 3', **axis_font)
        plt.show()
        