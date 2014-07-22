from xml.dom import minidom

class FeatureLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
    
class IndoorLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""
    
class OutdoorLabel(object):
    
    def __init__(self):
        self.id = ""
        self.name = ""
        self.type = ""

class LabelManager(object):
    
    def __init__(self):
        
        self.mapFile = None
        self.mapWidth = 0
        self.mapHeight = 0
        self.worldFile = None
        
        self.features = []
        self.indoors = []
        self.outdoors = []
        
        
    def loadFile(self, filename):
        
        xmldoc = minidom.parse(filename)
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
            self.features.append(f_l)
            
        for indoorLabel in indoorLabels:
            i_l = IndoorLabel()
            self.indoors.append(i_l)
            
        for outdoorLabel in outdoorLabels:
            o_l = OutdoorLabel()
            self.outdoors.append(o_l)
        
        
        
        
        

        