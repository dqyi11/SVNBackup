'''
Created on Jan 15, 2015

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

scores = np.loadtxt('data1.txt')

print scores

pathNum = len(scores)
objectiveNum = 3
        
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')

ax1.scatter(scores[0,0], scores[1,1], scores[2,2], c='g', marker='p')
for i in range(pathNum):
    if i < objectiveNum:
        ax1.scatter(scores[i,0], scores[i,1], scores[i,2], c ='b', marker='s')
    else:
        ax1.scatter(scores[i,0], scores[i,1], scores[i,2], c = 'r', marker='o')

fig2 = plt.figure()     
ax2 = fig2.add_subplot(111)
for i in range(pathNum):
    if i == 0 or i == 1:
        ax2.plot(scores[i,0], scores[i,1],'bs')
    else:
        ax2.plot(scores[i,0], scores[i,1],'ro')
        
fig3 = plt.figure()     
ax3 = fig3.add_subplot(111)
for i in range(pathNum):
    if i == 0 or i == 2:
        ax3.plot(scores[i,0], scores[i,2],'bs')
    else:
        ax3.plot(scores[i,0], scores[i,2],'ro')
        
fig4 = plt.figure()     
ax4 = fig4.add_subplot(111)
for i in range(pathNum):
    if i == 1 or i == 2:
        ax4.plot(scores[i,1], scores[i,2],'bs')
    else:
        ax4.plot(scores[i,1], scores[i,2],'ro')
        

plt.show()
        
        