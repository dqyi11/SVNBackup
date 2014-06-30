from PyQt4 import QtGui, QtCore
from HexagonalMap import *
from HexagonalMapState import *

class HexaMapWidget(QtGui.QWidget):

    def __init__(self, x_num, y_num, side, orientation):
        super(HexaMapWidget, self).__init__()
        self.hexamap = HexagonalMap(x_num, y_num, side, orientation)
        self.hexamapState = HexagonalMapState(self.hexamap)     
        self.initUI()
        
    def initUI(self):
        intCellWidth = int(self.hexamap.cellWidth)
        intCellHeight = int(self.hexamap.cellHeight)
        intMapWidth = int(self.hexamap.width)
        intMapHeight = int(self.hexamap.height)
        if self.hexamap.orientation == "FLAT":
            intMapHeight += self.hexamapState.defaultEdgeWidth*2
        elif self.hexamap.orientation == "POINTY":
            intMapWidth += self.hexamapState.defaultEdgeWidth*2
        self.setMinimumSize(intCellWidth, intCellHeight)
        self.resize(intMapWidth, intMapHeight)
        #print "W:" + str(self.width()) + " " + str(self.height())
        self.show()
        
    def mousePressEvent(self, e):
        if e.button() == QtCore.Qt.LeftButton:
            x_pos = e.pos().x()
            y_pos = e.pos().y()
            hexIdx = self.hexamap.findHex(x_pos, y_pos)
            if hexIdx != None:
                hexIdxList = self.hexamap.getHexesByRadius(hexIdx, 1)
                print "checking .. " + str(hexIdx)
                for nhex in hexIdxList:
                    print nhex
                    print self.hexamap.isConnected(hexIdx, nhex)
                    self.hexamapState.addActiveCells(nhex)
                self.update()
        
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        self.drawHexaMap(qp)
        qp.end()
        
    def drawHexaMap(self, qp):
        pen = QtGui.QPen(self.hexamapState.defaultEdgeColor)
        pen.setWidth(self.hexamapState.defaultEdgeWidth)        
        qp.setPen(pen)
        
        for i in range(self.hexamap.x_num):
            for j in range(self.hexamap.y_num):
                polygon = QtGui.QPolygonF()
                #print "i " + str(i) + " - j " + str(j) 
                hx = self.hexamap.hexes[i][j]
                for k in range(6):
                    pt = QtCore.QPointF(hx.points[k][0], hx.points[k][1])
                    polygon.append(pt)
                    #print pt
                brush = QtGui.QBrush(self.hexamapState.getHexColor(i,j))
                qp.setBrush(brush)
                qp.drawPolygon(polygon)
                
        brush = QtGui.QBrush(self.hexamapState.defaultActiveCellColor)
        qp.setBrush(brush)
        
        for cell in self.hexamapState.activeCells:
            hx = self.hexamap.getHex(cell[0], cell[1])
            if hx!=None:
                polygon = QtGui.QPolygonF()
                for k in range(6):
                    pt = QtCore.QPointF(hx.points[k][0], hx.points[k][1])
                    polygon.append(pt)
                qp.drawPolygon(polygon)
            
        
                
                
        
        