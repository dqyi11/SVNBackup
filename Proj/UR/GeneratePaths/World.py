'''
Created on Jul 30, 2015

@author: daqing_yi
'''
from xml.dom import minidom
from shapely.geometry import Polygon

class Object(object):

    def __init__(self):
        self.type = ""
        self.name = ""
        self.polygon = []  
        self.shape = None 
        self.center = None
        self.bounding = None

class World(object):

    def __init__(self, name=""):
        self.name = name
        self.width = 0
        self.height = 0
        self.objects = []
        
        self.init = None
        self.goal = None
        
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
            points = obj.getElementsByTagName("point")
            exteriors = []
            for p in points:
                x = int(p.getAttribute("x"))
                y = int(p.getAttribute("y"))
                new_obj.polygon.append([x,y])
                exteriors.append((x,y))
            new_obj.shape = Polygon(new_obj.polygon)
            new_obj.center = new_obj.shape.centroid
            new_obj.bounding = new_obj.shape.bounds
            self.objects.append(new_obj)
        
