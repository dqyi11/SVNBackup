'''
Created on Jan 15, 2015

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

LOAD_FILE = 'MORRTstar02-score.txt'
scores = np.loadtxt(LOAD_FILE)

print scores

pathNum = len(scores)
objectiveNum = 3
        
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')

'''
# interpolation spline with scipy 
from scipy import interpolate 

tck0 = interpolate.bisplrep(scores[:,0], scores[:,1], scores[:,2]) 
xmin = np.min(scores[:,0])
xmax = np.max(scores[:,0])
ymin = np.min(scores[:,1])
ymax = np.max(scores[:,1])
xnew,ynew = np.mgrid[xmin:xmax:70j,ymin:ymax:70j] 
znew = interpolate.bisplev(xnew[:,0],ynew[0,:],tck0) 
ax1.plot_surface(xnew, ynew, znew, linewidth=0.0, color='y', alpha = 0.25)
'''

ax1.scatter(scores[0,0], scores[1,1], scores[2,2], c='g', marker='p')
for i in range(pathNum):
    if i < objectiveNum:
        ax1.scatter(scores[i,0], scores[i,1], scores[i,2], c ='b', marker='s')
    else:
        ax1.scatter(scores[i,0], scores[i,1], scores[i,2], c = 'r', marker='o')
        
ax1.set_xlabel('Objective 1',fontsize=20)
ax1.set_ylabel('Objective 2',fontsize=20)
ax1.set_zlabel('Objective 3',fontsize=20)
        
plt.show()
        
        