#!/usr/bin/env python

from TurtlebotPlanner import *
import numpy as np
import matplotlib
matplotlib.use("WXAgg")
import matplotlib.pyplot as plt

target = TurtlebotTarget()
target.pos['x'] = 10
target.pos['y'] = 10

planner = TurtlebotPlanner(0.1)
planner.currentTarget = target
planner.plan(1.0)

fig = plt.figure()
ax = fig.add_subplot(111)
for t in range(len(planner.wayPoints)):
    p = planner.wayPoints[t]
    o = planner.wayOrientations[t]
    ax.scatter(p[0], p[1], color='red')
    ax.arrow(p[0],p[1], np.cos(o), np.sin(o))
    print p[0]
    print p[1]    
plt.show()    


