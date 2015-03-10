'''
Created on Mar 3, 2015

@author: daqing_yi
'''

from PathManager import *

equalChars = ['A0-1', 'A1-0', 'A1-1', 'A0-0']

def compareChar(nameA, nameB):
    if nameA == nameB:
        return True
    if (nameA in equalChars)==True and (nameB in equalChars)==True:
        return True
    return False
    

def shortenString(strPath):
    
    newStrPath = []
    if len(strPath)==0:
        return newStrPath
    newStrPath.append(strPath[0])
    
    for i in range(1, len(strPath)):
        if len(newStrPath) > 0 and compareChar(newStrPath[len(newStrPath)-1], strPath[i])==True:
            newStrPath.pop()
        else:
            newStrPath.append(strPath[i])
    return newStrPath

def compareStringPath(strPath, refStrPath, completeCompare=False):
        
    s_strPath = shortenString(strPath)
    s_refStrPath = shortenString(refStrPath)
    
    s_strPath_len = len(s_strPath)
    s_refStrPath_len = len(s_refStrPath)
    if s_strPath_len==0:
        if s_refStrPath_len == 0:
            return True
        else:
            return False
    
    if s_strPath_len > s_refStrPath_len:
        return False
    
    if completeCompare==True:
        if s_strPath_len != s_refStrPath_len:
            return False
    
    for i in range(len(s_strPath)):
        if compareChar(s_strPath[i], s_refStrPath[i]) == False:
            return False           
    return True


def merge(pairs):
    
    classes = {}
    
    classIdxCnt = 0
    for p in pairs:
        
        cstr0 = getClass(p[0], classes)
        cstr1 = getClass(p[1], classes)
        
        if cstr0 == None and cstr1 == None:
            # create a class
            classIdx = classIdxCnt
            classes[classIdx] = []
            classes[classIdx].append(p[0])
            classes[classIdx].append(p[1])
            classIdxCnt += 1
        elif cstr0 == None:
            # add to class of 1
            classes[cstr1].append(p[0])
        elif cstr1 == None:
            # add to class of 0
            classes[cstr0].append(p[1])
        else:
            if cstr1 != cstr0:
                # Merge
                for ch in classes[cstr1]:
                    classes[cstr0].append(ch)
                classes[cstr1] = []
            
    for cstr in classes.keys():
        if len(classes[cstr])==0:
            del classes[cstr]
        
    
    return classes

def getClass(string, classes):
    
    for cstr in classes.keys():
        if string in classes[cstr]:
            return cstr
    return None

if __name__ == '__main__':
    
    pathStrings = []
    
    FILENAME = 'classes01.txt'
    
    with open(FILENAME, 'r') as fp:
        data = fp.readlines()
        for line in data:
            exec 'pathStrings.append(' + line + ')'
             
    mgr = PathManager(None)
    
    print pathStrings
    
    pairs = []
    for i in range(len(pathStrings)-1):
        for j in range(i+1, len(pathStrings)):
            refPathStr = pathStrings[i]
            compathStr = pathStrings[j]
            if compareStringPath(compathStr, refPathStr, True)==True:
                print "Found Equivalence"
                strA = mgr.getString(compathStr) 
                strB = mgr.getString(refPathStr)
                print strA
                print strB
                
                pairs.append([strA, strB])
                
    classes = merge(pairs)
    
    print "CLASS NUM " + str(len(classes.keys()))
    for cstr in classes.keys():
        print classes[cstr]
        
        
    