'''
Created on Oct 18, 2013

@author: daqing_yi
'''

if __name__ == '__main__':
    
    idx = []
    currentScores = []
    maxScores = []
    exploredNum = []
    
    expandingNodeVals = []
    
    idxCnt = 0
    for line in open('data2.txt'):
        
        if idxCnt % 2 == 0:
            expandingNodeVals.append(float(line));        
        else:
            elements = line.split(',');
            idx.append(int(elements[0]));
            currentScores.append(float(elements[1]));
            maxScores.append(float(elements[2]));
            exploredNum.append(int(elements[3]));
            
        idxCnt = idxCnt + 1;    
            
    print expandingNodeVals;
    print idx;
    print currentScores;
    print maxScores;
    print exploredNum;        