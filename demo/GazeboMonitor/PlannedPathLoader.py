from xml.dom import minidom
import numpy as np

class PlannedPath(object):
    
    def __init__(self):        
        self.waypoints = []
        self.wayorientations = []
        self.length = 0
        
    def addWaypoint(self, point):
        self.waypoints.append(point)
        
    def update(self):
        self.length = len(self.waypoints)
        for t in range(self.length-1):
            deltaX = self.waypoints[t+1][0] - self.waypoints[t][0]
            deltaY = self.waypoints[t+1][1] - self.waypoints[t][1]
            orientation = np.arctan2(deltaY, deltaX)
            self.wayorientations.append(orientation)
        self.wayorientations.append(self.wayorientations[t-1])    
        
        print self.wayorientations
        

class PlannedPathLoader(object):
    
    def __init__(self):
        self.file = None
        self.path = None
        self.mapFile = None
        
        
    def load(self, filename):
        self.file = filename
        
        xmldoc = minidom.parse(filename)
        mapFileElement = xmldoc.getElementsByTagName('path')
        self.mapFile = mapFileElement[0].getAttribute('map')
        waypts = mapFileElement[0].getElementsByTagName('position')
        
        print self.mapFile
        
        self.path = PlannedPath()
        
        for pt in waypts:
            pos_x = float(pt.getAttribute('pos_x'))
            pos_y = float(pt.getAttribute('pos_y'))
            self.path.addWaypoint([pos_x, pos_y])
        
        self.path.update()    
        