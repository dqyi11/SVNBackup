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
            r = np.random.randint(3,5)
            self.radius = [r, r]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "person":
            r = np.random.randint(4,5)
            self.radius = [r, r]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "car":
            rw = np.random.randint(8,10)
            rh = np.random.randint(12,16)
            self.radius = [rw, rh]
            self.orientation = np.random.randint(0, 360)
        elif self.type == "tree":
            r = np.random.randint(3,6)
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
            rw = np.random.randint(12,12)
            rh = np.random.randint(1,2)
            self.radius = [rw, rh]
            self.orientation = np.random.randint(0, 360)

class World(object):

    def __init__(self, name=""):
        self.name = name
        self.width = 0
        self.height = 0
        self.objects = []
        
        self.init = None
        self.goal = None
        
    def initGoal(self):
        self.init = None
        self.goal = None
                
        for obj in self.objects:
            if obj.type == "robot":
                self.init = obj.center
        while self.goal == None:
            idx = np.random.randint(len(self.objects))
            if self.objects[idx].type != "robot":
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
            self.objects.append(new_obj)
        
    def dumpXML(self, filename):
        
        xmldoc = minidom.Document()
        root = xmldoc.createElement("world")
        root.setAttribute( "width", str(self.width) )
        root.setAttribute( "height", str(self.height) )
        xmldoc.appendChild(root)
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