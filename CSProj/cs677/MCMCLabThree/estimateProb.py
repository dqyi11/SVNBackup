'''
Created on 2013-6-9

@author: Walter
'''
import numpy as np

if __name__ == '__main__':
    
    observationData = []
    # add observation nodes
    for line in open('samples_for_wiki.txt'):
        instance = dict([])
        exec('instance='+line)
        observationData.append(instance)
        
    sampleNum = len(observationData)
    
    bCnt = 0
    eCnt = 0
    aCnt = np.zeros(2)
    abeCnt = np.zeros((2,2))
    beCnt = np.zeros((2,2))
    jaCnt = np.zeros(2)
    maCnt = np.zeros(2)
    
    for obs in observationData:
        if obs["Burglary"]==True:
            bCnt += 1
        if obs["Earthquake"]==True:
            eCnt += 1
        
        if obs["Burglary"]==True and obs["Earthquake"]==True:
            beCnt[0,0] += 1
            if obs["Alarm"]==True:
                abeCnt[0,0] += 1
                
        if obs["Burglary"]==True and obs["Earthquake"]==False:
            beCnt[0,1] += 1
            if obs["Alarm"]==True:
                abeCnt[0,1] += 1
                
        if obs["Burglary"]==False and obs["Earthquake"]==True:
            beCnt[1,0] += 1
            if obs["Alarm"]==True:
                abeCnt[1,0] += 1
                
        if obs["Burglary"]==False and obs["Earthquake"]==False:
            beCnt[1,1] += 1
            if obs["Alarm"]==True:
                abeCnt[1,1] += 1
        
        if obs["Alarm"]==True: 
            aCnt[0] += 1
            if obs["JohnCalls"]==True:
                jaCnt[0] += 1
            if obs["MaryCalls"]==True:
                maCnt[0] += 1
                
        if obs["Alarm"]==False: 
            aCnt[1] += 1
            if obs["JohnCalls"]==True:
                jaCnt[1] += 1
            if obs["MaryCalls"]==True:
                maCnt[1] += 1
                 
    print beCnt
       
    bProb = float(bCnt) / float(sampleNum)
    eProb = float(eCnt) / float(sampleNum)
    aProb = abeCnt / beCnt
    jProb = jaCnt / aCnt
    mProb = maCnt / aCnt
    
    print bProb
    print eProb
    print aProb
    print jProb
    print mProb