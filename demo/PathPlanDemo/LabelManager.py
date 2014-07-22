from xml.dom import minidom

class FeatureLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
        self.pos = None
    
class IndoorLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
        self.vertices = []
    
class OutdoorLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
        self.vertices = []

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
        
        
    def loadFile(self, filename):
        
        self.labelFile = filename        
        xmldoc = minidom.parse(self.labelFile)
        
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
        outdoorLabels = indoors.getElementsByTagName('Outdoor')
        
        for featureLabel in featureLabels:
            f_l = FeatureLabel()
            f_l.id = featureLabel.getAttribute("ID")
            f_l.name = featureLabel.getAttribute("Name")
            f_l.type = featureLabel.getAttribute("Type")
            pos = featureLabel.getAttribute("Pos")
            print self.parsePos(pos)
            self.features.append(f_l)
            
        for indoorLabel in indoorLabels:
            i_l = IndoorLabel()
            i_l.id = indoorLabel.getAttribute("ID")
            i_l.name = indoorLabel.getAttribute("Name")
            vertices = indoorLabel.getElementsByTagName('Vertices')[0].getElementsByTagName('Vertex')
            for vex in vertices:
                pos = vex.getAttribute("Pos")
                print self.parsePos(pos)
            self.indoors.append(i_l)
            
        for outdoorLabel in outdoorLabels:
            o_l = OutdoorLabel()
            o_l.id = outdoorLabel.getAttribute("ID")
            o_l.name = outdoorLabel.getAttribute("Name")
            o_l.type = outdoorLabel.getAttribute("Type")
            vertices = outdoorLabel.getElementsByTagName('Vertices')[0].getElementsByTagName('Vertex')
            for vex in vertices:
                pos = vex.getAttribute("Pos")
                print self.parsePos(pos)
            self.outdoors.append(o_l)
            
    def parsePos(self, str):
        x, y = str.unformat('{X={0},Y={1}}')
        return [int(x), int(y)]
        
        
        
        
        
        

        