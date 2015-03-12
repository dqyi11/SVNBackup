'''
Created on Jan 15, 2015

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt

LOAD_FILE = 'MORRTstar00-score.txt'
LOAD_FILE = 'MOPath01-score.txt'
LOAD_FILE = 'MORRTstar01-1-score.txt'
LOAD_FILE = 'MORRTstar03-score.txt'
scores = np.loadtxt(LOAD_FILE)

print scores

pathNum = len(scores)
objectiveNum = 2
        
fig = plt.figure()
ax = fig.add_subplot(111)
for i in range(pathNum):
    if i < objectiveNum:
        ax.plot(scores[i,0], scores[i,1],'bs')
    else:
        ax.plot(scores[i,0], scores[i,1],'ro')
        
ax.tick_params(axis='both', which='major', labelsize=20)
ax.tick_params(axis='both', which='minor', labelsize=18)
ax.set_xlabel("objective 1", fontsize=30)
ax.set_ylabel("objective 2", fontsize=30)

plt.show()
        
        