#!/usr/bin/env python

from TurtlebotPlanner import *
from TurtlebotController import *
from PlannerMonitor import *
import numpy as np
import matplotlib
import sys
matplotlib.use("WXAgg")
import matplotlib.pyplot as plt

target = TurtlebotTarget()
target.pos['x'] = 10
target.pos['y'] = 10

planner = TurtlebotPlanner(0.5)
planner.currentTarget = target

app = QtGui.QApplication(sys.argv)

monitor = PlannerMonitor([800,600],20)
monitor.planner = planner

planner.update()
planner.plan(1.0)

monitor.startMonitor(0.01)

sys.exit(app.exec_())

'''
fig = plt.figure()
ax = fig.add_subplot(111)
for t in range(len(planner.waypoints)):
    p = planner.waypoints[t]
    o = planner.wayorientations[t]
    ax.scatter(p[0], p[1], color='red')
    ax.arrow(p[0],p[1], np.cos(o), np.sin(o))
    print p[0]
    print p[1]    
plt.show() 
''' 

