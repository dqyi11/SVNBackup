'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import cv2

from ObstacleMgr import *
from RegionMgr import *
from TopologicalGraph import *
import sympy.geometry as symgeo
import sympy.core as symcore
import shapely.geometry as shpgeo
import numpy 


class WorldMapMgr(object):

    def __init__(self):
        self.obstacles = []
        self.regions = []
        self.region_colors = []
        
        self.subsegments = []
        
        self.centralPoint = None
    
    def load(self, mapfile):
        self.mapfile = mapfile
        self.mapImgData = cv2.imread(mapfile)
        self.height = self.mapImgData.shape[0]
        self.width = self.mapImgData.shape[1]
        self.mapImgGrayData = cv2.cvtColor(self.mapImgData, cv2.COLOR_BGR2GRAY)
        
        self.sampleWidthScale = self.width/5
        self.sampleHeightScale = self.height/5
        
        ret, thresh = cv2.threshold(255-self.mapImgGrayData, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cont in contours:
            obs = ObstacleMgr(cont, len(self.obstacles), self)
            self.obstacles.append(obs)
            
        self.regionNum = len(self.obstacles)*2
        
        self.init()
        self.initSegments()
        self.process()

        
        
    def init(self):
        # select random point for each obstacle
        for obs in self.obstacles:
            obs.bk = obs.samplePosition()
            
        self.obsBkPairLines = []
        for i in range(len(self.obstacles)):
            for j in range(i+1, len(self.obstacles)):
                bk1 = self.obstacles[i].bk
                bk2 = self.obstacles[j].bk
                pline = symgeo.Line(symgeo.Point(bk1[0], bk1[1]),symgeo.Point(bk2[0], bk2[1]))
                self.obsBkPairLines.append(pline)
                
        
        # select central point c
        self.centralPoint = (self.width/2, self.height/2)
        foundCP = False
        while foundCP == False:
            if (self.isInObstacle(self.centralPoint) == False) and (self.isInObsBkLinePair(self.centralPoint)==False):
                foundCP = True
            else:
                cpX = np.random.normal()*(self.sampleWidthScale) + self.width/2
                cpY = np.random.random()*(self.sampleHeightScale) + self.height/2
                self.centralPoint = (int(cpX), int(cpY))
                #print "Resampling " + str(self.centralPoint)
                
        # init four boundary line
        self.boundary_lines = []
        self.x_axis = symgeo.Line(symgeo.Point(0,0), symgeo.Point(self.width-1,0))
        self.y_axis = symgeo.Line(symgeo.Point(0,0), symgeo.Point(0, self.height-1))
        self.x_high = symgeo.Line(symgeo.Point(0,self.height-1), symgeo.Point(self.width-1, self.height-1))
        self.y_high = symgeo.Line(symgeo.Point(self.width-1, 0), symgeo.Point(self.width-1, self.height-1))
        self.boundary_lines.append(self.x_axis)
        self.boundary_lines.append(self.y_high)
        self.boundary_lines.append(self.x_high)
        self.boundary_lines.append(self.y_axis)
        
        # init lines from center point to four corners
        self.center_corner_lines_info = []
        self.center_corner_lines_info.append( [ (0,0), numpy.arctan2(float(-self.centralPoint[1]), float(-self.centralPoint[0])) ] )
        self.center_corner_lines_info.append( [ (0,self.height), numpy.arctan2(float(self.height-self.centralPoint[1]), float(-self.centralPoint[0])) ] )
        self.center_corner_lines_info.append( [ (self.width,self.height), numpy.arctan2(float(self.height-self.centralPoint[1]), float(self.width-self.centralPoint[0])) ] )
        self.center_corner_lines_info.append( [ (self.width,0), numpy.arctan2(float(-self.centralPoint[1]), float(self.width-self.centralPoint[0])) ] )
        for ccl_info in self.center_corner_lines_info:
            if ccl_info[1] < 0:
                ccl_info[1] += 2*numpy.pi
        self.center_corner_lines_info.sort( key=lambda x: x[1], reverse=False )
        #print "CENTER CORNER LINES "
        #print self.center_corner_lines_info
        
        self.ray_info_list = []
        # init alpah and beta segments
        for obs in self.obstacles:
            obs.alpha_ray = symgeo.Ray(symgeo.Point(obs.bk[0], obs.bk[1]), symgeo.Point(self.centralPoint[0],self.centralPoint[1]))
            obs.beta_ray = symgeo.Ray(symgeo.Point(obs.bk[0], obs.bk[1]), symgeo.Point(2*obs.bk[0]-self.centralPoint[0], 2*obs.bk[1]-self.centralPoint[1])) 
            
            a_pt = self.findIntersectionWithBoundary(obs.alpha_ray)
            #print str(obs.alpha_ray) + " --> " + str(a_pt)
            b_pt = self.findIntersectionWithBoundary(obs.beta_ray)
            #print str(obs.beta_ray) + " --> " + str(b_pt)
                
            alpha_ray_rad = numpy.arctan(float(obs.alpha_ray.slope))
            if obs.alpha_ray.p1.x > obs.alpha_ray.p2.x:
                alpha_ray_rad += numpy.pi
            if alpha_ray_rad < 0:
                alpha_ray_rad += 2*numpy.pi
            alpha_ray_info = (obs.idx, 'A', alpha_ray_rad)
            
            beta_ray_rad = numpy.arctan(float(obs.beta_ray.slope))
            if obs.beta_ray.p1.x > obs.beta_ray.p2.x:
                beta_ray_rad += numpy.pi
            if beta_ray_rad < 0:
                beta_ray_rad += 2*numpy.pi                
            beta_ray_info = (obs.idx, 'B', beta_ray_rad)
            
                        
            obs.alpha_seg = None
            obs.beta_seg = None
            if a_pt != None:
                alpha_seg = shpgeo.LineString([obs.bk, (a_pt.x, a_pt.y)])
                obs.alpha_seg = LineSegmentMgr(alpha_seg, 'A', alpha_ray_rad, obs)
            if b_pt != None:
                beta_seg = shpgeo.LineString([obs.bk, (b_pt.x, b_pt.y)])
                obs.beta_seg = LineSegmentMgr(beta_seg, 'B', beta_ray_rad, obs)
            
            #print "ALPHA RAY SLOPE " + str(alpha_ray_info)
            #print "BETA RAY SLOPE " + str(beta_ray_info)
            self.ray_info_list.append(alpha_ray_info)
            self.ray_info_list.append(beta_ray_info)
              
            obs.alpha_seg_info = ( (a_pt.x, a_pt.y), alpha_ray_rad )
            obs.beta_seg_info = ( (b_pt.x, b_pt.y), beta_ray_rad )  
              
                
        self.ray_info_list.sort(key=lambda x: x[2], reverse=False)
        #print self.ray_info_list
    
    def initSegments(self):    
    
        for obs in self.obstacles:
            
            for obs_ref in self.obstacles:
                
                # check alpha seg with obstacles
                isIntersect = obs.alpha_seg.line_seg.intersects(obs_ref.polygon)
                if isIntersect == True:
                    alpha_intsecs = obs.alpha_seg.line_seg.intersection(obs_ref.polygon)
                    #alpha_intsecs = obs_ref.polygon.intersection(obs.alpha_seg.line_seg)
                    
                    #print "ALPHA: " + str(obs.idx) + " + " + str(obs_ref.idx) #+ " = " + str(alpha_intsecs)
                    if alpha_intsecs.type == "LineString":
                        if alpha_intsecs.is_empty == False:
                            for c in list(alpha_intsecs.coords):
                                cpos = (c[0], c[1])
                                cdist = numpy.sqrt((obs.bk[0]-cpos[0])**2+(obs.bk[1]-cpos[1])**2)
                                obs.alpha_obs_intsecs.append(cpos)
                                obs.alpha_obs_intsecs_info.append((obs_ref.idx, 'A', cpos, cdist))
                    elif alpha_intsecs.type == "MultiLineString":
                        for int_line in alpha_intsecs:
                            if int_line.is_empty == False:
                                for c in list(int_line.coords):
                                    cpos = (c[0], c[1])
                                    cdist = numpy.sqrt((obs.bk[0]-cpos[0])**2+(obs.bk[1]-cpos[1])**2)
                                    obs.alpha_obs_intsecs.append(cpos)
                                    obs.alpha_obs_intsecs_info.append((obs_ref.idx, 'A', cpos, cdist))            
                
                isIntersect = obs.beta_seg.line_seg.intersects(obs_ref.polygon)
                if isIntersect == True:
                    # check beta seg with obstacles
                    beta_intsecs = obs.beta_seg.line_seg.intersection(obs_ref.polygon)
                    
                    #print "BETA: " + str(obs.idx) + " + " + str(obs_ref.idx) #+ " = " + str(beta_intsecs)
                    if beta_intsecs.type == "LineString":
                        if beta_intsecs.is_empty == False:
                            for c in list(beta_intsecs.coords):
                                cpos = (c[0], c[1])
                                cdist = numpy.sqrt((obs.bk[0]-cpos[0])**2+(obs.bk[1]-cpos[1])**2)
                                obs.beta_obs_intsecs.append(cpos)
                                obs.beta_obs_intsecs_info.append((obs_ref.idx, 'B', cpos, cdist))
                    elif beta_intsecs.type == "MultiLineString":
                        for int_line in beta_intsecs:
                            if int_line.is_empty == False:
                                for c in list(int_line.coords):
                                    cpos = (c[0], c[1])
                                    cdist = numpy.sqrt((obs.bk[0]-cpos[0])**2+(obs.bk[1]-cpos[1])**2)
                                    obs.beta_obs_intsecs.append(cpos)
                                    obs.beta_obs_intsecs_info.append((obs_ref.idx, 'B', cpos, cdist))
            
            obs.alpha_obs_intsecs_info.sort(key=lambda x: x[3], reverse=False)
            obs.beta_obs_intsecs_info.sort(key=lambda x: x[3], reverse=False)
            obs.alpha_seg.load(obs.alpha_obs_intsecs_info)
            obs.beta_seg.load(obs.beta_obs_intsecs_info)
            
        for obs in self.obstacles:
            for linSeg in obs.alpha_seg.sub_segs:
                self.subsegments.append(linSeg)
            for linSeg in obs.beta_seg.sub_segs:
                self.subsegments.append(linSeg)
                
        self.subsegments.sort(key=lambda x: x.rad, reverse=False)
            
            
    def process(self):
        
        # split the region
        self.regions = []
        self.region_colors = []
        for i in range(len(self.ray_info_list)-1):
            ray1_info = self.ray_info_list[i]
            ray2_info = self.ray_info_list[i+1]
            region = RegionMgr(ray1_info, ray2_info, i, self)
            region.ref_rad = (ray1_info[2] + ray2_info[2])/2
            self.regions.append(region)
            #rndVal = numpy.random.randint(0, 256, 3)
            #self.region_colors.append((rndVal[0], rndVal[1], rndVal[2]))
        ray1_info = self.ray_info_list[len(self.ray_info_list)-1]
        ray2_info = self.ray_info_list[0]
        region = RegionMgr(ray1_info, ray2_info, len(self.regions), self)
        region.ref_rad = (ray1_info[2] + ray2_info[2] + 2*np.pi)/2
        self.regions.append(region)
        #rndVal = numpy.random.randint(0, 256, 3)
        #self.region_colors.append((rndVal[0], rndVal[1], rndVal[2], 100))    
        
        for i in range(len(self.ray_info_list)):
            # pick two neighbors here
            ray_info = self.ray_info_list[i]
            ray_rad = ray_info[2]

            delta_x, delta_y = self.calcCheckDelta(ray_rad)
         
            obs = self.obstacles[ray_info[0]]
            if ray_info[1] == 'A':
                seg = obs.alpha_seg
            else:
                seg = obs.beta_seg

            
            for linseg in seg.sub_segs:
                # check each sub seg of a seg line
                linseg.checkPosA = (linseg.midpoint[0]+delta_x, linseg.midpoint[1]+delta_y)
                linseg.checkPosB = (linseg.midpoint[0]-delta_x, linseg.midpoint[1]-delta_y)
                
                regionA, regionB = self.findNeighborRegion(linseg.rad)
                print "SEG RAD " + str(linseg.rad) + " REG A " + str(regionA.ref_rad) + " REG B " + str(regionB.ref_rad)
                
                linseg.checkRegionA = regionA
                linseg.checkRegionB = regionB
                
                print linseg.name
                #print test_points
                iline = shpgeo.LineString([linseg.checkPosA, linseg.checkPosB])
                sgA = self.isLineIntersectedWithRegion(regionA.subregions, iline)
                #sgA =  self.isPointsInRegion(regionA.subregions, test_points)
                if sgA != None:
                    linseg.regionAInfo = sgA
                    print "      " + sgA.getName()
                else:
                    print "   " + regionA.name
                
                iline = shpgeo.LineString([linseg.checkPosA, linseg.checkPosB])
                sgB = self.isLineIntersectedWithRegion(regionB.subregions, iline)    
                #sgB = self.isPointsInRegion(regionB.subregions, test_points) 
                if sgB != None:
                    linseg.regionBInfo = sgB
                    print "      " + sgB.getName()
                else:
                    print "   " + regionB.name
                    
    def getTopologicalGraph(self):
        
        g = TopologicalGraph()
        for reg in self.regions:
            for sr in reg.subregions:
                g.addNode(sr.getName())
        for seg in self.subsegments:
            nodeA = g.findNode(seg.regionAInfo.getName())
            nodeB = g.findNode(seg.regionBInfo.getName())
            g.addEdge(nodeA, nodeB, seg.name)
            
        return g
                    
    def findNeighborRegion(self, rad):
        regionA = None
        regionB = None
        if self.regions[len(self.regions)-1].ref_rad - 2*np.pi <= rad  and rad < self.regions[0].ref_rad:
            regionA = self.regions[len(self.regions)-1]
            regionB = self.regions[0]
        elif self.regions[len(self.regions)-1] <= rad and rad < self.regions[0].ref_rad + 2*np.pi:
            regionA = self.regions[len(self.regions)-1]
            regionB = self.regions[0]
        else:
            for i in range(len(self.regions)-1):
                if self.regions[i].ref_rad <= rad and rad < self.regions[i+1].ref_rad:
                    regionA = self.regions[i]
                    regionB = self.regions[i+1]
            
        return regionA, regionB
        
                    
    def calcCheckDelta(self, ray_rad):
        rad_len = 2
        delta_x = rad_len * np.cos(ray_rad+np.pi/2)
        delta_y = rad_len * np.sin(ray_rad+np.pi/2)
        
        return delta_x, delta_y
    
    def isLineIntersectedWithRegion(self, subregions, iline):
        for sg in subregions:
            if sg.polygon.intersects(iline):
                return sg
        return None
        
    
    def isPointsInRegion(self, subregions, points):
        for p in points:
            pnt = shpgeo.Point(p)
            for sg in subregions:
                if sg.polygon.contains(pnt):
                    return sg
        return None
                
    def isInObsBkLinePair(self, pos):
        
        pPos = symgeo.Point(pos[0], pos[1])
        for bkline in self.obsBkPairLines:
            if bkline.contains(pPos):
                return True
        return False 
    
    def isInObstacle(self, pos):   
        cPoint = shpgeo.Point(pos[0], pos[1])
        for obs in self.obstacles:
            if obs.polygon.contains(cPoint):
                return True
        return False
        
 
    def findIntersectionWithBoundary(self, a_ray):
        for bl in self.boundary_lines:
            intsecs = symgeo.intersection(bl, a_ray)
            #print str(a_ray) + " + " + str(bl) + " = " + str(intsecs)
            if len(intsecs) > 0:
                intsec = intsecs[0]
                if intsec.x >=0 and intsec.x <self.width and intsec.y >=0 and intsec.y <self.height:
                    return intsec
        return None

