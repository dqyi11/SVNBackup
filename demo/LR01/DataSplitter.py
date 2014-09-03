import csv
import random
import numpy as np

class DataSplitter(object):


    def __init__(self, filename):
        self.testNum = 0
        self.trainNum = 0    
        self.dataSize = 0
        self.dataList = []   
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.dataSize += 1
                self.dataList.append(row)
    
    def dump(self, filename, testRatio):
        self.testNum = int(testRatio * self.dataSize)
        self.trainNum = self.dataSize - self.testNum
        
        intList = np.arange(self.dataSize)
        with open(filename+"-train.csv", 'w') as trainFile:
            #trainWriter = csv.writer(trainFile, delimiter=',')
            for i in range(self.trainNum):
                dataLen = len(self.dataList[intList[i]])
                dataTxt = ""
                if dataLen > 1:
                    for j in range(dataLen-1):
                        dataTxt += self.dataList[intList[i]][j] + ","
                    dataTxt += self.dataList[intList[i]][dataLen-1] + "\n"
                #trainWriter.writerow(self.dataList[intList[i]])
                else:
                    dataTxt += self.dataList[intList[i]][0] + "\n"
                trainFile.write(dataTxt)
                
        with open(filename+"-test.csv", 'w') as testFile:
            #testWriter = csv.writer(testFile, delimiter=',')
            for i in range(self.trainNum, self.dataSize):
                #testWriter.writerow(self.dataList[intList[i]])
                dataLen = len(self.dataList[intList[i]])
                dataTxt = ""
                if dataLen > 1:
                    for j in range(dataLen-1):
                        dataTxt += self.dataList[intList[i]][j] + ","
                    dataTxt += self.dataList[intList[i]][dataLen-1] + "\n"
                #trainWriter.writerow(self.dataList[intList[i]])
                else:
                    dataTxt += self.dataList[intList[i]][0] + "\n"
                testFile.write(dataTxt)
            
            
        