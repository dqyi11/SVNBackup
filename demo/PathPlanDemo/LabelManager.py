from xml.dom import minidom
import re

class FeatureLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
        self.pos = None
        
    def getLabel(self):
        return "ID: " + self.id + " NAME: " + self.name + " TYPE: " + self.type + " POS: " + str(self.pos)
    
class IndoorLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
        self.vertices = []
        
    def getLabel(self):
        string = "ID: " + str(self.id) + " NAME: " + str(self.name) + " TYPE: " + self.type + " \n"
        string += "VERTICES: " 
        for v in self.vertices:
            string += str(v) + ", "
        return string            
    
class OutdoorLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
        self.vertices = []
        
    def getLabel(self):
        string = "ID: " + str(self.id) + " NAME: " + str(self.name) + " TYPE: " + self.type + " \n"
        string += "VERTICES: " 
        for v in self.vertices:
            string += str(v) + ", "
        return string            
    
        
class EnemyLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.pos = None
        
    def getLabel(self):
        return "ID: " + self.id + " NAME: " + self.name + " POS: " + str(self.pos)

class LabelManager(object):
    
    def __init__(self):
        
        self.labelFile = None
        self.mapFile = None
        self.mapWidth = 0
        self.mapHeight = 0
        self.worldFile = None
        
        self.features = []
        self.indoors = []
        self.outdoors = []
        self.enemies = []
        
        
    def loadFile(self, filename):
        
        self.labelFile = filename    
        #print self.labelFile
        f = open(self.labelFile)    
        xmldoc = minidom.parseString(f.read())
        
        mapLabel = xmldoc.getElementsByTagName('MapLabel')[0]
        self.mapFile = mapLabel.getAttribute("MapFile")
        self.mapWidth = int(mapLabel.getAttribute("MapWidth"))
        self.mapHeight = int(mapLabel.getAttribute("MapHeight"))
        self.worldFile = mapLabel.getAttribute("WorldFile")
        
        features = xmldoc.getElementsByTagName('features')[0]
        featureLabels = features.getElementsByTagName('Feature')
        indoors = xmldoc.getElementsByTagName('indoors')[0]
        indoorLabels = indoors.getElementsByTagName('Indoor')
        outdoors = xmldoc.getElementsByTagName('outdoors')[0]
        outdoorLabels = outdoors.getElementsByTagName('Outdoor')
        enemies = xmldoc.getElementsByTagName('enemies')[0]
        enemyLabels = enemies.getElementsByTagName('Enemy')
        
        for featureLabel in featureLabels:
            f_l = FeatureLabel()
            f_l.id = featureLabel.getAttribute("ID")
            f_l.name = featureLabel.getAttribute("Name")
            f_l.type = featureLabel.getAttribute("Type")
            pos = self.parsePos(featureLabel.getAttribute("Pos"))
            #print pos
            f_l.pos = pos
            self.features.append(f_l)
            
        for indoorLabel in indoorLabels:
            i_l = IndoorLabel()
            i_l.id = indoorLabel.getAttribute("ID")
            i_l.name = indoorLabel.getAttribute("Name")
            vertices = indoorLabel.getElementsByTagName('Vertices')[0].getElementsByTagName('Vertex')
            for vex in vertices:
                pos = self.parsePos(vex.getAttribute("Pos"))
                #print pos
                i_l.vertices.append(pos)
            self.indoors.append(i_l)
            
        for outdoorLabel in outdoorLabels:
            o_l = OutdoorLabel()
            o_l.id = outdoorLabel.getAttribute("ID")
            o_l.name = outdoorLabel.getAttribute("Name")
            o_l.type = outdoorLabel.getAttribute("Type")
            vertices = outdoorLabel.getElementsByTagName('Vertices')[0].getElementsByTagName('Vertex')
            for vex in vertices:
                pos = self.parsePos(vex.getAttribute("Pos"))
                #print pos
                o_l.vertices.append(pos)
            self.outdoors.append(o_l)
            
        for enemyLabel in enemyLabels:
            e_l = EnemyLabel()
            e_l.id = enemyLabel.getAttribute("ID")
            e_l.name = enemyLabel.getAttribute("Name")
            pos = self.parsePos(enemyLabel.getAttribute("Pos"))
            #print pos
            e_l.pos = pos
            self.enemies.append(e_l)
            
    def parsePos(self, strPos):        
        p = re.compile('\d+')
        ms = p.findall(str(strPos))
        return [int(ms[0]), int(ms[1])]
    
    def printLabels(self):
        
        print "Width: " + str(self.mapWidth) + " Height: " + str(self.mapHeight)
        print "Label: " + str(self.labelFile)
        print "Map: " + str(self.mapFile)
        print "World: " + str(self.worldFile)
        
        print "FEATURE "
        for f in self.features:
            print f.getLabel()
        print "INDOOR "
        for i in self.indoors:
            print i.getLabel()
        print "OUTDOOR "
        for o in self.outdoors:
            print o.getLabel()
        print "ENEMY "
        for e in self.enemies:
            print e.getLabel()
        
        
        
        

        