from shapely.geometry import Polygon
from shapely import ops
import copy

class PolygonManager(object):
       
    def merge(self, polygons):
        newPolygons = copy.deepcopy(polygons)
        
        while True:
            findIntersect = False
            
            polyNum = len(newPolygons)
            for i in range(polyNum):
                for j in range(polyNum):
                    if i!=j:
                        poly1=newPolygons[i]
                        poly2=newPolygons[j]
                        if poly2.intersects(poly1)==True:
                            pols = [poly1, poly2]
                            new_pol = ops.cascaded_union(pols)
                            newPolygons[i] = new_pol
                            newPolygons.remove(poly2)
                            findIntersect = True
                            break
                        else:
                            continue
                break
            
            if findIntersect==False:
                break
        
        return newPolygons
        
        
        
            
                
                    
        
        