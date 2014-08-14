from xml.dom import minidom

class PathManager(object):

    def __init__(self, map, mapFile, scale):
        self.map = map
        self.mapFile = mapFile
        self.scale = scale
        
    def dumpPath(self, path, filename):
        
        doc = minidom.Document()
        root = doc.createElement("path")
        root.setAttribute('map', self.mapFile)
        doc.appendChild(root)
        
        for p in path:
            node_p = doc.createElement("position")
            node_p.setAttribute("idx_x", str(p[0]))
            node_p.setAttribute("idx_y", str(p[1]))
            h = self.map.getHex(p[0],p[1])
            node_p.setAttribute("pos_x", str(h.center[0]))
            node_p.setAttribute("pos_y", str(h.center[1]))
            root.appendChild(node_p)
            
        doc.toprettyxml()
        doc.writexml( open(filename, 'w'), indent="", addindent="", newl='')
        