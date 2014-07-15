'''
Created on Apr 24, 2014

@author: walter
'''

from xml.dom import minidom
from PyQt4 import QtGui, QtCore
import math
import numpy as np
from shapely.geometry import Polygon
from PolygonManager import *


class enemyObject(object):
    
    def __init__(self, name):
        self.name = name
        self.pos = []


class modelObject(object):
    
    def __init__(self, name):
        self.name = name
        self.map_pos = []
        self.pos = []
        self.statics = []

class staticObject(object):
    
    def __init__(self):
        self.pos = []        
        self.col_pos = []
        self.col_size = []
        self.type = ""
        
class worldInfo(object):
    
    def __init__(self):
        self.width = 0
        self.height = 0
        self.numberOfPolygon = 0
        self.graph = graphInfo()
        
class graphInfo(object):
    
    def __init__(self):
        self.edges = []
        self.vertices = []
        
    def addVertex(self, x, y):
        newIdx = len(self.vertices)
        intX = int(x)
        intY = int(y)
        radius = np.sqrt(x**2+y**2)
        angle = np.arctan2(y,x)*180/np.pi
        if angle < 0:
            angle += 360.0
        vex = vertexInfo(newIdx)
        vex.x = intX
        vex.y = intY
        vex.radius = radius
        vex.angle = angle
        
        self.vertices.append(vex)
        
    def addEdge(self, aX, aY, bX, bY):
        newIdx = len(self.edges)
        
        intAX = int(aX)
        intAY = int(aY)
        intBX = int(bX)
        intBY = int(bY)
        
        a_radius = np.sqrt(aX**2+aY**2)
        b_radius = np.sqrt(bX**2+bY**2)
        a_angle = np.arctan2(aY,aX)*180/np.pi
        b_angle = np.arctan2(bY,bX)*180/np.pi
        if a_angle < 0:
            a_angle += 360
        if b_angle < 0:
            b_angle += 360
            
        ed = edgeInfo(newIdx)
        ed.a_x = intAX
        ed.a_y = intAY
        ed.b_x = intBX
        ed.b_y = intBY
        ed.a_radius = a_radius
        ed.a_angle = a_angle
        ed.b_radius = b_radius
        ed.b_angle = b_angle
        
        self.edges.append(ed)
        
        
class edgeInfo(object):
    
    def __init__(self, index):
        self.idx = index
        self.a_angle = 0.0
        self.a_radius = 0.0
        self.a_x = 0
        self.a_y = 0
        
        self.b_angle = 0.0
        self.b_radius = 0.0
        self.b_x = 0
        self.b_y = 0
        
class vertexInfo(object):
    
    def __init__(self, index):
        self.idx = index
        self.angle = 0.0
        self.radius = 0.0
        self.x = 0
        self.y = 0
        

class WorldFileReader(QtGui.QWidget):

    def __init__(self, file, scale=5):
        
        self.worldSize = []
        
        self.models = []
        self.enemies = []        
        
        self.parseFile(file)
        self.filename = file
        self.scale = scale
        super(WorldFileReader, self).__init__()
        self.initUI()
        
        self.worldInfo = None
        
    def initUI(self):
        if len(self.worldSize) != 0:
            self.width = int(self.scale*self.worldSize[0])
            self.height = int(self.scale*self.worldSize[1])
            self.setGeometry(300, 300, self.width, self.height)
        self.show()
        
    def findModel(self, name):
        
        for model in self.models:
            if model.name == name:
                return model
        return None
    
    def findEnemy(self, name):
        
        for enemy in self.enemies:
            if enemy.name == name:
                return enemy
        return None
        
    def parseBit(self, data):
        data_array =  data.split(' ')
        float_array = []
        for d in data_array:
            float_array.append(float(d))
        return float_array
    
    def dump(self, outfile):
        
        img = QtGui.QImage(QtCore.QSize(self.width, self.height), QtGui.QImage.Format_ARGB32_Premultiplied)
        img.fill(QtCore.Qt.white)
        qp = QtGui.QPainter(img)
        #qp.begin()
        color = QtGui.QColor(0,0,0)
        qp.setPen(color)
        qp.setBrush(color)
        
        for model in self.models:
            #print "a: " + model.name
            
            for link in model.statics:
                
                #posX = link.pos[0] + model.pos[0] + model.map_pos[0] + self.worldSize[0]/2.0 
                #posY = link.pos[1] + model.pos[1] + model.map_pos[1] + self.worldSize[0]/2.0
                posX = link.pos[0] + model.pos[0] + self.worldSize[0]/2.0 
                posY = link.pos[1] + model.pos[1] + self.worldSize[1]/2.0
                
                #print link.pos
                #print link.col_pos
                #print link.col_size
                
                rot = link.pos[5] + model.map_pos[5]
                #rect = QtCore.QRectF(posX*self.scale,posY*self.scale,link.col_size[0]*self.scale,link.col_size[1]*self.scale) 
                #print rect
                #qp.drawRect(rect)    
                ULx = posX+(link.col_size[0]*math.cos(rot)/2.0)-(link.col_size[1]*math.sin(rot)/2.0)
                ULy = posY+(link.col_size[1]*math.cos(rot)/2.0)+(link.col_size[0]*math.sin(rot)/2.0)
                URx = posX-(link.col_size[0]*math.cos(rot)/2.0)-(link.col_size[1]*math.sin(rot)/2.0)
                URy = posY+(link.col_size[1]*math.cos(rot)/2.0)-(link.col_size[0]*math.sin(rot)/2.0)
                BLx = posX+(link.col_size[0]*math.cos(rot)/2.0)+(link.col_size[1]*math.sin(rot)/2.0)
                BLy = posY-(link.col_size[1]*math.cos(rot)/2.0)+(link.col_size[0]*math.sin(rot)/2.0)
                BRx = posX-(link.col_size[0]*math.cos(rot)/2.0)+(link.col_size[1]*math.sin(rot)/2.0)
                BRy = posY-(link.col_size[1]*math.cos(rot)/2.0)-(link.col_size[0]*math.sin(rot)/2.0)    
                
                ULx = ULx*self.scale
                ULy = ULy*self.scale
                URx = URx*self.scale
                URy = URy*self.scale
                BLx = BLx*self.scale
                BLy = BLy*self.scale
                BRx = BRx*self.scale
                BRy = BRy*self.scale
                                
                qp.drawPolygon(QtCore.QPointF(ULx,ULy), QtCore.QPointF(URx,URy),QtCore.QPointF(BRx,BRy),QtCore.QPointF(BLx,BLy))
        
        for enemy in self.enemies:
            
            yellow_color = QtGui.QColor(0,255,255)
            qp.setPen(yellow_color)
            qp.setBrush(yellow_color)
            
            enemy_posX = enemy.pos[0] + self.worldSize[0]/2.0 
            enemy_posY = enemy.pos[1] + self.worldSize[1]/2.0 
            
            qp.drawRect(int(enemy_posX*self.scale), int(enemy_posY*self.scale), 5, 5)
        
        qp.end()
        img.save(outfile, 'PNG')
    
    def paintEvent(self, e):

        if len(self.models)==0:
            return
        
        qp = QtGui.QPainter()
        qp.begin(self)
        
        color = QtGui.QColor(0,0,0)
        qp.setPen(color)
        qp.setBrush(color)
        
        for model in self.models:
            #print "a: " + model.name
            
            for link in model.statics:
                
                #posX = link.pos[0] + model.pos[0] + model.map_pos[0] + self.worldSize[0]/2.0 
                #posY = link.pos[1] + model.pos[1] + model.map_pos[1] + self.worldSize[0]/2.0
                posX = link.pos[0] + model.pos[0] + self.worldSize[0]/2.0 
                posY = link.pos[1] + model.pos[1] + self.worldSize[0]/2.0
                
                #print link.pos
                #print link.col_pos
                #print link.col_size
                
                rot = link.pos[5] + model.map_pos[5]
                #rect = QtCore.QRectF(posX*self.scale,posY*self.scale,link.col_size[0]*self.scale,link.col_size[1]*self.scale) 
                #print rect
                #qp.drawRect(rect)    
                ULx = posX+(link.col_size[0]*math.cos(rot)/2.0)-(link.col_size[1]*math.sin(rot)/2.0)
                ULy = posY+(link.col_size[1]*math.cos(rot)/2.0)+(link.col_size[0]*math.sin(rot)/2.0)
                URx = posX-(link.col_size[0]*math.cos(rot)/2.0)-(link.col_size[1]*math.sin(rot)/2.0)
                URy = posY+(link.col_size[1]*math.cos(rot)/2.0)-(link.col_size[0]*math.sin(rot)/2.0)
                BLx = posX+(link.col_size[0]*math.cos(rot)/2.0)+(link.col_size[1]*math.sin(rot)/2.0)
                BLy = posY-(link.col_size[1]*math.cos(rot)/2.0)+(link.col_size[0]*math.sin(rot)/2.0)
                BRx = posX-(link.col_size[0]*math.cos(rot)/2.0)+(link.col_size[1]*math.sin(rot)/2.0)
                BRy = posY-(link.col_size[1]*math.cos(rot)/2.0)-(link.col_size[0]*math.sin(rot)/2.0)    
                
                ULx = ULx*self.scale
                ULy = ULy*self.scale
                URx = URx*self.scale
                URy = URy*self.scale
                BLx = BLx*self.scale
                BLy = BLy*self.scale
                BRx = BRx*self.scale
                BRy = BRy*self.scale
                                
                qp.drawPolygon(QtCore.QPointF(ULx,ULy), QtCore.QPointF(URx,URy),QtCore.QPointF(BRx,BRy),QtCore.QPointF(BLx,BLy))
        
        for enemy in self.enemies:
            
            yellow_color = QtGui.QColor(0,255,255)
            qp.setPen(yellow_color)
            qp.setBrush(yellow_color)
            
            enemy_posX = enemy.pos[0] + self.worldSize[0]/2.0 
            enemy_posY = enemy.pos[1] + self.worldSize[1]/2.0 
            
            qp.drawRect(int(enemy_posX*self.scale), int(enemy_posY*self.scale), 5, 5)
                    
        qp.end()
        
        
    def parseFile(self, filename):
        
        self.models = []
        xmldoc = minidom.parse(filename)
        models = xmldoc.getElementsByTagName('model')
        
        for model in models:
            statics = model.getElementsByTagName("static")
            if len(statics)==0:
                continue
            
            if int(statics[0].firstChild.data) == 1:
                #print model.getAttribute("name")
                if model.getAttribute("name") != "ground_plane":
                    mod_obj = modelObject(model.getAttribute("name"))
                    self.models.append(mod_obj)
            else:
                name = model.getAttribute("name")
                if name == "pioneer2dx":
                    enemy = enemyObject(name)
                    self.enemies.append(enemy)
            
        for model in models:
            #print model.getAttribute("Name")
            name = model.getAttribute("name")
            modelx = self.findModel(name)
            statics = model.getElementsByTagName("static")
            
            if len(statics) == 0:
                if modelx != None:
                    model_pos = model.getElementsByTagName("pose")[0]
                    modelx.map_pos = self.parseBit(model_pos.firstChild.data)
                else:
                    enemy = self.findEnemy(name)
                    if enemy != None:
                        enemy_pos = model.getElementsByTagName("pose")[0]
                        enemy.pos = self.parseBit(enemy_pos.firstChild.data)
                        #print enemy.pos
    
            elif int(statics[0].firstChild.data) == 1:
                #print model.getAttribute("name")
                
                if model.getAttribute("name") == "ground_plane":
                    collision = model.getElementsByTagName("collision")[0]
                    coll_size = collision.getElementsByTagName("size")[0]
                    self.worldSize = self.parseBit(coll_size.firstChild.data)
                else:
                    
                    for d in model.childNodes:
                        if d.localName == "pose":
                            modelx.pos = self.parseBit(d.firstChild.data)
                    
                    links = model.getElementsByTagName("link")
                    for link in links:
                        so = staticObject()
                        modelx.statics.append(so)
                        
                        #print link.getAttribute("name")
                        for node in link.childNodes:
                            if node.nodeName == "collision":
                                col_pos = node.getElementsByTagName("pose")[0]
                                so.col_pos = self.parseBit(col_pos.firstChild.data)
                                
                                col_size = node.getElementsByTagName("size")[0]
                                so.col_size = self.parseBit(col_size.firstChild.data)
                            elif node.nodeName == "pose":
                                so.pos = self.parseBit(node.firstChild.data)


    def initWorldInfo(self):
        self.worldInfo =  worldInfo()
        self.worldInfo.width = self.worldSize[0]*self.scale
        self.worldInfo.height = self.worldSize[1]*self.scale
        self.polygons = []
        for model in self.models:
            for link in model.statics:
                #self.worldInfo.numberOfPolygon += 1
                        
                              
                #posX = link.pos[0] + model.pos[0] + model.map_pos[0] + self.worldSize[0]/2.0 
                #posY = link.pos[1] + model.pos[1] + model.map_pos[1] + self.worldSize[0]/2.0
                posX = link.pos[0] + model.pos[0] + self.worldSize[0]/2.0 
                posY = link.pos[1] + model.pos[1] + self.worldSize[0]/2.0
                
                #print link.pos
                #print link.col_pos
                #print link.col_size
                
                rot = link.pos[5] + model.map_pos[5]
                #rect = QtCore.QRectF(posX*self.scale,posY*self.scale,link.col_size[0]*self.scale,link.col_size[1]*self.scale) 
                #print rect
                #qp.drawRect(rect)    
                ULx = posX+(link.col_size[0]*math.cos(rot)/2.0)-(link.col_size[1]*math.sin(rot)/2.0)
                ULy = posY+(link.col_size[1]*math.cos(rot)/2.0)+(link.col_size[0]*math.sin(rot)/2.0)
                URx = posX-(link.col_size[0]*math.cos(rot)/2.0)-(link.col_size[1]*math.sin(rot)/2.0)
                URy = posY+(link.col_size[1]*math.cos(rot)/2.0)-(link.col_size[0]*math.sin(rot)/2.0)
                BLx = posX+(link.col_size[0]*math.cos(rot)/2.0)+(link.col_size[1]*math.sin(rot)/2.0)
                BLy = posY-(link.col_size[1]*math.cos(rot)/2.0)+(link.col_size[0]*math.sin(rot)/2.0)
                BRx = posX-(link.col_size[0]*math.cos(rot)/2.0)+(link.col_size[1]*math.sin(rot)/2.0)
                BRy = posY-(link.col_size[1]*math.cos(rot)/2.0)-(link.col_size[0]*math.sin(rot)/2.0)    
                
                ULx = ULx*self.scale
                ULy = ULy*self.scale
                URx = URx*self.scale
                URy = URy*self.scale
                BLx = BLx*self.scale
                BLy = BLy*self.scale
                BRx = BRx*self.scale
                BRy = BRy*self.scale
                
                #self.worldInfo.graph.addVertex(ULx, ULy)
                #self.worldInfo.graph.addVertex(URx, URy)
                #self.worldInfo.graph.addVertex(BLx, BLy)
                #self.worldInfo.graph.addVertex(BRx, BRy)
                
                #self.worldInfo.graph.addEdge(ULx, ULy, URx, URy)
                #self.worldInfo.graph.addEdge(URx, URy, BRx, BRy)
                #self.worldInfo.graph.addEdge(BRx, BRy, BLx, BLy)
                #self.worldInfo.graph.addEdge(BLx, BLy, ULx, ULy)
                
                self.polygons.append(Polygon([(ULx, ULy),(URx, URy),(BRx, BRy), (BLx, BLy)]))
        self.polygons = PolygonManager().merge(self.polygons)
        
        self.worldInfo.numberOfPolygon = len(self.polygons)
        for poly in self.polygons:
            vexNum = len(poly.exterior.coords)
            for i in range(vexNum-1):
                self.worldInfo.graph.addVertex(poly.exterior.coords[i][0], poly.exterior.coords[i][1])
                self.worldInfo.graph.addVertex(poly.exterior.coords[i+1][0], poly.exterior.coords[i+1][1])
                self.worldInfo.graph.addEdge(poly.exterior.coords[i][0], poly.exterior.coords[i][1], poly.exterior.coords[i+1][0], poly.exterior.coords[i+1][1])        
            
            self.worldInfo.graph.addEdge(poly.exterior.coords[vexNum-1][0], poly.exterior.coords[vexNum-1][1], poly.exterior.coords[0][0], poly.exterior.coords[0][1])    
    
                
                
    def dumpWorldInfo(self, filename):
        
        #print "dumping.."        
        
        doc = minidom.Document()
        root = doc.createElement("World")
        doc.appendChild(root)
        
        x_max_node = doc.createElement("X_MAX")
        x_max_node_val = doc.createTextNode(str(int(self.worldInfo.width)))
        x_max_node.appendChild(x_max_node_val)
        y_max_node = doc.createElement("Y_MAX")
        y_max_node_val = doc.createTextNode(str(int(self.worldInfo.height)))
        y_max_node.appendChild(y_max_node_val)
        root.appendChild(x_max_node)
        root.appendChild(y_max_node)
        
        
        #print "edges " + str(len(self.worldInfo.graph.edges))       
        
        edges_node = doc.createElement("edges")
        for edge in self.worldInfo.graph.edges:
            #print str(edge.idx)
            edge_node = doc.createElement("Edge")
            edge_name_node = doc.createElement("name")
            
            edge_a_node = doc.createElement("a")
            edge_a_x_node = doc.createElement("x")
            edge_a_x_node_val = doc.createTextNode(str(edge.a_x))
            edge_a_x_node.appendChild(edge_a_x_node_val)
            edge_a_y_node = doc.createElement("y")
            edge_a_y_node_val = doc.createTextNode(str(edge.a_y))
            edge_a_y_node.appendChild(edge_a_y_node_val)
            edge_a_angle_node = doc.createElement("angle")
            edge_a_angle_node_val = doc.createTextNode(str(edge.a_angle))
            edge_a_angle_node.appendChild(edge_a_angle_node_val)
            edge_a_radius_node = doc.createElement("radius")
            edge_a_radius_node_val = doc.createTextNode(str(edge.a_radius))
            edge_a_radius_node.appendChild(edge_a_radius_node_val)
            edge_a_name_node = doc.createElement("name")
            edge_a_name_node_val = doc.createTextNode(str(edge.idx))
            edge_a_name_node.appendChild(edge_a_name_node_val)
            edge_a_node.appendChild(edge_a_x_node)
            edge_a_node.appendChild(edge_a_y_node)
            edge_a_node.appendChild(edge_a_angle_node)
            edge_a_node.appendChild(edge_a_radius_node)
            edge_a_node.appendChild(edge_a_name_node)            
            
            edge_b_node = doc.createElement("b")
            edge_b_x_node = doc.createElement("x")
            edge_b_x_node_val = doc.createTextNode(str(edge.b_x))
            edge_b_x_node.appendChild(edge_b_x_node_val)
            edge_b_y_node = doc.createElement("y")
            edge_b_y_node_val = doc.createTextNode(str(edge.b_y))
            edge_b_y_node.appendChild(edge_b_y_node_val)
            edge_b_angle_node = doc.createElement("angle")
            edge_b_angle_node_val = doc.createTextNode(str(edge.b_angle))
            edge_b_angle_node.appendChild(edge_b_angle_node_val)
            edge_b_radius_node = doc.createElement("radius")
            edge_b_radius_node_val = doc.createTextNode(str(edge.b_radius))
            edge_b_radius_node.appendChild(edge_b_radius_node_val)
            edge_b_name_node = doc.createElement("name")
            edge_b_name_node_val = doc.createTextNode(str(edge.idx))
            edge_b_name_node.appendChild(edge_b_name_node_val)
            edge_b_node.appendChild(edge_b_x_node)
            edge_b_node.appendChild(edge_b_y_node)
            edge_b_node.appendChild(edge_b_angle_node)
            edge_b_node.appendChild(edge_b_radius_node)
            edge_b_node.appendChild(edge_b_name_node)              
            
            edge_distance_node = doc.createElement("distance")
            edge_distance_node_val = doc.createTextNode(str(0))
            edge_distance_node.appendChild(edge_distance_node_val)
            edge_name_node = doc.createElement("name")
            edge_name_node_val = doc.createTextNode(str(edge.idx))
            edge_name_node.appendChild(edge_name_node_val)
            edge_was_seen_node = doc.createElement("wasSeen")
            #edge_was_seen_node_val = doc.createTextNode(str(False))
            edge_was_seen_node_val = doc.createTextNode("false")
            edge_was_seen_node.appendChild(edge_was_seen_node_val)
            
            edge_node.appendChild(edge_a_node)
            edge_node.appendChild(edge_b_node)
            edge_node.appendChild(edge_distance_node)
            edge_node.appendChild(edge_name_node)
            edge_node.appendChild(edge_was_seen_node)            
            
            edges_node.appendChild(edge_node)
        
        #print "vertices " + str(len(self.worldInfo.graph.vertices))    
        num_poly_node = doc.createElement("numPolygons")
        num_poly_node_val = doc.createTextNode(str(self.worldInfo.numberOfPolygon))
        num_poly_node.appendChild(num_poly_node_val)
        root.appendChild(num_poly_node)

        vertices_node = doc.createElement("vertices")        
        for vertex in self.worldInfo.graph.vertices:
            
            #print str(vertex.idx)
            vertex_node = doc.createElement("Vertex")
            
            vertex_name_node = doc.createElement("name")
            vertex_name_node_val = doc.createTextNode(str(vertex.idx))
            vertex_name_node.appendChild(vertex_name_node_val)
            vertex_angle_node = doc.createElement("angle")
            vertex_angle_node_val = doc.createTextNode(str(vertex.angle))
            vertex_angle_node.appendChild(vertex_angle_node_val)
            vertex_radius_node = doc.createElement("radius")
            vertex_radius_node_val = doc.createTextNode(str(vertex.radius))
            vertex_radius_node.appendChild(vertex_radius_node_val)
            vertex_x_node = doc.createElement("x")
            vertex_x_node_val = doc.createTextNode(str(vertex.x))
            vertex_x_node.appendChild(vertex_x_node_val)
            vertex_y_node = doc.createElement("y")
            vertex_y_node_val = doc.createTextNode(str(vertex.y))
            vertex_y_node.appendChild(vertex_y_node_val)
            
            vertex_node.appendChild(vertex_name_node)
            vertex_node.appendChild(vertex_angle_node)
            vertex_node.appendChild(vertex_radius_node)
            vertex_node.appendChild(vertex_x_node)
            vertex_node.appendChild(vertex_y_node)
            
            vertices_node.appendChild(vertex_node)
        
        root.appendChild(edges_node)
        root.appendChild(vertices_node)
        
        '''
        enemies_node = doc.createElement("enemies")
        for enemy in self.enemies:
            enemy_node = doc.createElement("enemy")
            enemy_node.setAttribute("pos_x",str(enemy.pos[0]));
            enemy_node.setAttribute("pos_y",str(enemy.pos[1]));
            enemies_node.appendChild(enemy_node)
        root.appendChild(enemies_node)
        '''
        #print "writing..."
        
        doc.writexml( open(filename, 'w'), indent="", addindent="", newl='')
        
        #doc.unlink()
    
            
    def dumpWorldObstacle(self, filename):
        
        #print "dumping.."        
        f = open(filename, 'w')
        f.write(str(self.worldInfo.width)+","+str(self.worldInfo.height)+"\n")
        
        for edge in self.worldInfo.graph.edges:
            f.write(str(edge.a_x)+","+str(edge.a_y)+","+str(edge.b_x)+","+str(edge.b_y)+"\n")
     
        
