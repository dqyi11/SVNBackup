import numpy as np
    
def calcHexaWidthHeight(side):
    # http://hexnet.org/content/hexagonal-geometry
    # h - short length
    # r - long length        
    h = np.sin(30*np.pi/180)*side
    r = np.cos(30*np.pi/180)*side
    return h, r


def isHexInList(hexIdxList, hexIdx):    
    for nxh in hexIdxList:
        if nxh[0]==hexIdx[0] and nxh[1]==hexIdx[1]:
            return True
    return False

def removeHexFromList(hexIdxList, hexIdx):
    hexIdxList.remove(hexIdx)
    
def calcHexDimension(width, height, side, orientation):
    x_num = 0
    y_num = 0    
    h, r = calcHexaWidthHeight(side)

    if orientation == "FLAT":
        cellWidth = side + h + h
        cellHeight = r + r
            
        x_num = int(width/(cellWidth+side))
        y_num = int(2*height/cellHeight)

    elif orientation == "POINTY":
        cellWidth = r + r
        cellHeight = side + h + h
        x_num = int(width/cellWidth)
        y_num = int(2*height/(cellHeight+side))
    
    return x_num, y_num
