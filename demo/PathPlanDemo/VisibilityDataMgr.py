import re
import numpy as np

class VisibilityDataMgr(object):

    def __init__(self):    
        self.filename = None

        self.width = 0
        self.height = 0
        self.hexSize = 0
        self.visData = []
        self.currentHexId = None
        self.visSumData = []
        
    def loadFile(self, filename):
        self.filename = filename
        
        f = open(filename, 'r')
        headStr = f.readline()
        self.width, self.height, self.hexSize = self.parseFilehead(headStr)
        print str(self.width)+"-"+str(self.height)+"-"+str(self.hexSize)
        f.readline()
        x = f.read()
        xs = x.split("\n\n\n")
        for j in range(self.height):
            xxs = xs[j].split("\n\n")
            for i in range(self.width):
                #print str(i) + " + " + str(j)
                #print xxs[i]
                self.visData.append(self.parseDataBlock(xxs[i]))
        #print len(xs)
        #print self.width*self.height
        #print xs[len(xs)-1]
        
        '''
        for j in range(self.height):
            for i in range(self.width):
                print str(i) + " + " + str(j)
                self.visData.append(self.parseDataBlock(xs[i+j*self.width]))
        '''

        #print xs[21]
        #print self.parseDataBlock(xs[21])
        
        f.close()
        
        #print self.visData[0]
        self.visSumData = np.zeros((self.width, self.height))
        for x in range(self.width):
            for y in range(self.height):
                self.visSumData[x,y]=np.sum(np.sum(self.visData[x+self.width*y],axis=0))
                
        minVal = self.visSumData.min()
        maxVal = self.visSumData.max()
        ran = float(maxVal - minVal)
        for i in range(self.width):
            for j in range(self.height):
                self.visSumData[i,j] =  (self.visSumData[i,j]-minVal)/ran
        

    def parseFilehead(self, headStr):
        m = re.match(r'Columns: (\d+), Rows: (\d+), HexSize: (\d+)', headStr)
        vals = m.groups()
        return int(vals[0]), int(vals[1]), int(vals[2])
    
    def parseDataBlock(self, blockStr, i=None, j=None):
        blocks = blockStr.split('\n')
        dataVals = np.zeros((self.width, self.height))
        
        if i!=None and j!=None:
            if blocks[0] != str(i)+","+str(j):
                return
            
        #print len(blocks)
        
        for rowIdx in range(self.height):
            #print str(rowIdx) + ":" + str(len(blocks))
            lineData = blocks[rowIdx+1]
            #print lineData
            uline = lineData.split(',')
            for colIdx in range(self.width):
                #print str(rowIdx)+"~"+str(colIdx)+"~"+str(len(uline))
                dataVals[colIdx, rowIdx] = float(uline[colIdx])
        return dataVals
    
    def dumpData(self, filename):
        
        f = open(filename, 'w')
        
        f.write("Columns: "+str(self.width) + ", Rows: " + str(self.height) + ", HexSize: " + str(self.hexSize) + "\n")
        
        for j in range(self.height):
            for i in range(self.width):
                f.write("\n")
                f.write(str(i)+","+str(j)+":\n")
                for rowIdx in range(self.height):
                    for colIdx in range(self.width):
                        #print self.visData[j*self.width+i]
                        dataVal = self.visData[j*self.width+i][colIdx,rowIdx]
                        if dataVal==0.0:
                            f.write("0")
                        else:
                            f.write(str(dataVal))
                        if colIdx < self.width-1:
                            f.write(",")
                    f.write("\n")
            f.write("\n")
        
        f.close()
        
    def getValue(self, x, y):        
        if self.currentHexId != None:
            listIdx = self.currentHexId[1]*self.width+self.currentHexId[0]
            return self.visData[listIdx][x, y]
        return 0.5
    
    def getValueByHexId(self, hexId, x, y):
        listIdx = hexId[1]*self.width+hexId[0]
        #print str(listIdx) + " / " + str(len(self.visData))
        return self.visData[listIdx][x, y]
    
    def randInit(self, width, height, hexSize):
        
        self.width = width
        self.height = height
        self.hexSize = hexSize
        
        self.visData = []
        for col in range(width):
            for row in range(height):
                data = np.random.random((width, height))
                self.visData.append(data)

            
        
        
    

        