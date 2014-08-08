from HexagonalMap import *


class Hex(object):

    def __init__(self, x, y, side, orientation, h, r):
        
        self.x = x
        self.y = y 
        self.side = side
        self.orientation = orientation
        self.points = []
        
        if self.orientation == "FLAT":
            # UpperLeft = 0, UpperRight = 1, MiddleRight = 2
            # BottomRight = 3, BottomLeft = 4, MiddleLeft = 5
            self.points.append([x, y])
            self.points.append([x+side, y])
            self.points.append([x+side+h, y+r])
            self.points.append([x+side, y+r+r])
            self.points.append([x, y+r+r])
            self.points.append([x-h, y+r])
        elif self.orientation == "POINTY":
            # Top = 0, UpperRight = 1, BottomRight = 2
            # Bottom = 3, BottomLeft = 4, TopLeft = 5
            self.points.append([x, y])
            self.points.append([x+r, y+h])
            self.points.append([x+r, y+side+h])
            self.points.append([x, y+side+h+h])
            self.points.append([x-r, y+side+h])
            self.points.append([x-r, y+h])
            
        self.center = [0, 0]
        centerX = 0.0
        centerY = 0.0
        for i in range(6):
            centerX += self.points[i][0]
            centerY += self.points[i][1]
        centerX = centerX / 6.0
        centerY = centerY / 6.0
        self.center[0] = centerX
        self.center[1] = centerY
        
    def insideHex(self, x, y):
        #http://astronomy.swin.edu.au/~pbourke/geometry/insidepoly/
        
        # Slick algorithm that checks if a point is inside a polygon.  Checks how may time a line
        # origination from point will cross each side.  An odd result means inside polygon.
        
        vexCnt = len(self.points)
        j = vexCnt - 1
        oddNodes = False
        
        for i in range(vexCnt):
            pt_i = self.points[i]
            pt_j = self.points[j]
            if (pt_i[1] < y and pt_j[1]>=y) or (pt_j[1] < y and pt_i[1] >= y):
                if pt_i[0] + (y-pt_i[1])*(pt_j[0]-pt_i[0])/(pt_j[1]-pt_i[1]) < x:  
                    oddNodes = not oddNodes
            j = i
        return oddNodes      
        
        
        
        
            
        
            
        
        
        
