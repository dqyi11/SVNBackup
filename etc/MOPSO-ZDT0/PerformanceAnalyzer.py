'''
Created on Jan 24, 2014

@author: daqing_yi
'''

import numpy as np;
import matplotlib.pyplot as plt;

class PerformanceAnalyzer(object):
    '''
    classdocs
    '''


    def __init__(self, files, folder, dataName, windowSize):
        '''
        Constructor
        '''
        self.dataName = dataName;
        self.rawDataList = [];
        self.folder = folder;
        self.windowSize = windowSize;
        
        for file in files:
            filename = folder + "\\" + file;
            f = open(filename, 'r');
            #print filename
            line = f.readline();
            #print line
            
            data = [];
            for l in line.split():
                data.append(float(l));
            self.rawDataList.append(data);
        
        #for data in self.rawDataList:
            #print data;    
        #print self.rawDataList;
        
    def genData(self):
        
        self.dataLen = len(self.rawDataList[0]);
        self.newDataList = [];
        
        for d in self.rawDataList:
            newData = []
            for t in range(self.dataLen):
                if t < self.windowSize:
                    start = 0;
                else:
                    start = t - self.windowSize + 1;
                if t > self.dataLen - self.windowSize:
                    windowLen = self.dataLen - t;
                else:
                    windowLen = self.windowSize; 
                newD = 0.0;
                for t1 in range(start, start + windowLen):
                    newD += d[t1];
                newD /= float(windowLen);
                newData.append(newD);
            self.newDataList.append(newData);
            #print d;
            #print newData;
                
        
        self.avData = np.zeros(self.dataLen);
        for t in range(self.dataLen):
            for data in self.newDataList:
                self.avData[t] += data[t];
            self.avData[t] /= float(len(self.rawDataList));
        
    def plot(self, name):
        
        fig = plt.figure();
        ax = fig.add_subplot(111);
        ax.plot(np.arange(self.dataLen), self.avData);
        ax.set_title(self.dataName);
        
        filename = self.folder + "//" + name + "-" + self.dataName + ".png"
        plt.savefig(filename)
        
    def dump(self, name):
        
        f1 = open(self.folder+"//"+ name + self.dataName + ".txt", "w");
        for data in self.avData:
            f1.write(str(data)+" ");
        f1.close();
            
            