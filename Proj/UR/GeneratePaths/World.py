'''
Created on Jul 30, 2015

@author: daqing_yi
'''
from xml.dom import minidom
from shapely.geometry import Polygon
import numpy as np

OBJ_TYPES = { "robot", "person", "car", "tree", "building", "chapel", "gate" }

class Object(object):

    def __init__(self):
        self.type = ""
        self.name = ""
        self.polygon = []
        self.center = None
        self.radius = None
        self.orientation = 0.0
        
    def randParam(self):
        if self.type == "robot":
            r = np.random.randint(50,60)
            self.radius = [r, r]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "person":
            r = np.random.randint(40,50)
            self.radius = [r, r]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "car":
            rw = np.random.randint(60,90)
            rh = np.random.randint(80,100)
            self.radius = [rw, rh]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "tree":
            r = np.random.randint(50,70)
            self.radius = [r, r]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "building":
            rw = np.random.randint(60,100)
            rh = np.random.randint(60,100)
            self.radius = [rw, rh]
            self.orientation = np.random.randint(0, 360)            
        elif self.type == "chapel":
            rw = np.random.randint(50,70)
            rh = np.random.randint(50,70)
            self.radius = [rw, rh]
            self.orientation = np.random.randint(0, 360)            
        elif self.type == "gate":
            rw = np.random.randint(60,80)
            rh = np.random.randint(50,60)
            self.radius = [rw, rh]
            self.orientation = np.random.randint(0, 360)

class World(object):

    def __init__(self, name=""):
        self.name = name
        self.width = 0
        self.height = 0
        self.objects = []
        self.robot = None
        
        self.init = None
        self.goal = None
        
    def initRobot(self):
        self.init = None
        self.goal = None
        self.init = self.robot.center
                
    def selectGoal(self, i=-1):
        if i>=0 and i<len(self.objects):
            self.goal = self.objects[i].center
        else:
            idx = np.random.randint(0, len(self.objects))
            self.goal = self.objects[idx].center
        
        
    def fromXML(self, filename):
        
        xmldoc = minidom.parse(filename)
        world = xmldoc.getElementsByTagName('world')[0]
        self.width = int(world.getAttribute("width"))
        self.height = int(world.getAttribute("height"))

        objects = world.getElementsByTagName("object")
        for obj in objects:
            new_obj = Object()
            new_obj.name = obj.getAttribute("name")
            new_obj.type = obj.getAttribute("type")
            cx = int(obj.getAttribute("x"))
            cy = int(obj.getAttribute("y"))
            cw = int(obj.getAttribute("w"))
            ch = int(obj.getAttribute("h"))
            new_obj.center = [cx, cy]
            new_obj.radius = [cw, ch]
            new_obj.orientation = int(obj.getAttribute("orientation"))
            points = obj.getElementsByTagName("point")
            exteriors = []
            for p in points:
                x = int(p.getAttribute("x"))
                y = int(p.getAttribute("y"))
                new_obj.polygon.append([x,y])
                exteriors.append((x,y))
            if len(points) > 0:
                new_obj.shape = Polygon(new_obj.polygon)
                #new_obj.center = new_obj.shape.centroid
                new_obj.bounding = new_obj.shape.bounds
            if new_obj.type == "robot":
                self.robot = new_obj
            else:    
                self.objects.append(new_obj)
        
    def dumpXML(self, filename):
        
        xmldoc = minidom.Document()
        root = xmldoc.createElement("world")
        root.setAttribute( "width", str(self.width) )
        root.setAttribute( "height", str(self.height) )
        xmldoc.appendChild(root)
        if self.robot != None:
            objChild = xmldoc.createElement("object")
            objChild.setAttribute("name", self.robot.name)
            objChild.setAttribute("type", self.robot.type)
            objChild.setAttribute("x", str(self.robot.center[0]))
            objChild.setAttribute("y", str(self.robot.center[1]))
            objChild.setAttribute("w", str(self.robot.radius[0]))
            objChild.setAttribute("h", str(self.robot.radius[1]))
            objChild.setAttribute("orientation", str(self.robot.orientation))
            for p in self.robot.polygon:
                pointChild = xmldoc.createElement("point")
                pointChild.setAttribute( "x", str(p[0]) )
                pointChild.setAttribute( "y", str(p[1]) )
                objChild.appendChild(pointChild)
            root.appendChild(objChild)
        for obj in self.objects:
            objChild = xmldoc.createElement("object")
            objChild.setAttribute("name", obj.name)
            objChild.setAttribute("type", obj.type)
            objChild.setAttribute("x", str(obj.center[0]))
            objChild.setAttribute("y", str(obj.center[1]))
            objChild.setAttribute("w", str(obj.radius[0]))
            objChild.setAttribute("h", str(obj.radius[1]))
            objChild.setAttribute("orientation", str(obj.orientation))
            for p in obj.polygon:
                pointChild = xmldoc.createElement("point")
                pointChild.setAttribute( "x", str(p[0]) )
                pointChild.setAttribute( "y", str(p[1]) )
                objChild.appendChild(pointChild)
            root.appendChild(objChild)
        
        xmldoc.writexml( open(filename, 'w'), indent="  ", addindent="  ", newl="\n" )
        xmldoc.unlink()