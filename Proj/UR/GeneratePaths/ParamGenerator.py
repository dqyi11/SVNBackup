from gmm import *
import numpy as np
from __builtin__ import True
from xml.dom import minidom

class ParamGenerator(object):

    def __init__(self, worldViz):
        
        self.worldViz = worldViz

    def generateParams(self, num, resolution=0.2):
        params = []        
        num_res = int(1/resolution)
        for i in range(num):
            param = Param()
            param.w = np.random.randint( -num_res , num_res+1, len(self.worldViz.world.objects)) * resolution
            param.scale = 140 * np.ones(len(self.worldViz.world.objects))           
            params.append(param)        
        return params    
    
    def getBooleanList(self, int_num):
        obj_num = len(self.worldViz.world.objects)
        bl = np.zeros(obj_num, np.bool)
        int_1 = int_num % (2**obj_num)
        for i in range(obj_num):
            if int_1%(2**i)==1:
                bl[i] = True
        return bl
    
    
    def dumpXML(self, param, filename):
        
        if filename == "":
            return
        
        xmldoc = minidom.Document()
        root = xmldoc.createElement("world")
        root.setAttribute( "width", str(self.worldViz.world.width) )
        root.setAttribute( "height", str(self.worldViz.world.height) )
        xmldoc.appendChild(root)
        
        robot_node =  xmldoc.createElement("robot")
        root.appendChild(robot_node)
        if self.worldViz.world.robot != None:
            objChild = xmldoc.createElement("object")
            objChild.setAttribute("name", self.worldViz.world.robot.name)
            objChild.setAttribute("type", self.worldViz.world.robot.type)
            objChild.setAttribute("x", str(self.worldViz.world.robot.center[0]))
            objChild.setAttribute("y", str(self.worldViz.world.robot.center[1]))
            objChild.setAttribute("w", str(self.worldViz.world.robot.radius[0]))
            objChild.setAttribute("h", str(self.worldViz.world.robot.radius[1]))
            objChild.setAttribute("orientation", str(self.worldViz.world.robot.orientation))
            for p in self.worldViz.world.robot.polygon:
                pointChild = xmldoc.createElement("point")
                pointChild.setAttribute( "x", str(p[0]) )
                pointChild.setAttribute( "y", str(p[1]) )
                objChild.appendChild(pointChild)
            robot_node.appendChild(objChild)
        
        objects_node = xmldoc.createElement("objects")
        root.appendChild(objects_node)
        idx = 0
        for obj in self.worldViz.world.objects:
            objChild = xmldoc.createElement("object")
            objChild.setAttribute("name", obj.name)
            objChild.setAttribute("type", obj.type)
            objChild.setAttribute("x", str(obj.center[0]))
            objChild.setAttribute("y", str(obj.center[1]))
            objChild.setAttribute("w", str(obj.radius[0]))
            objChild.setAttribute("h", str(obj.radius[1]))
            objChild.setAttribute("orientation", str(obj.orientation))
            objChild.setAttribute("weight", str(param.w[idx]) )
            objChild.setAttribute("scale", str(param.scale[idx]) )
            for p in obj.polygon:
                pointChild = xmldoc.createElement("point")
                pointChild.setAttribute( "x", str(p[0]) )
                pointChild.setAttribute( "y", str(p[1]) )
                objChild.appendChild(pointChild)
            objects_node.appendChild(objChild)
            idx += 1
        
        xmldoc.writexml( open(filename, 'w'), indent="  ", addindent="  ", newl="\n" )
        xmldoc.unlink()        