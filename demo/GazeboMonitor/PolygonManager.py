from shapely.geometry import Polygon
from shapely import ops
import copy

class PolygonManager(object):
       
    def merge(self, polygons):
        newPolygons = copy.deepcopy(polygons)
        
        while True:
            
            #print "new loop"
            if False==self.hasIntersection(newPolygons):
                return newPolygons
            
            merged = False
            polyNum = len(newPolygons)
            for i in range(polyNum-1):
                for j in range(i+1, polyNum):

                    poly1=newPolygons[i]
                    poly2=newPolygons[j]
                    
                    if poly2.intersects(poly1)==True:
                        #print str(i)+":"+str(j)+" intersect " + str(len(newPolygons))
                        pols = [poly1, poly2]
                        new_pol = ops.cascaded_union(pols)
                        newPolygons[i] = new_pol
                        newPolygons.remove(poly2)
                        merged = True
                        break
                    #else:
                        #print str(i)+":"+str(j)+" no intersect " + str(len(newPolygons))
                if merged==True:
                    break
        
        return newPolygons
    
    def hasIntersection(self, polygons):           
        polyNum = len(polygons)

        for i in range(polyNum-1):
            for j in range(i+1, polyNum):
                poly1=polygons[i]
                poly2=polygons[j]
                if poly2.intersects(poly1)==True:
                    #print "Found between " + str(i) + " - " + str(j)
                    return True
                
        #print "No intersect"
        return False
        
        
        
        
            
                
                    
        
        