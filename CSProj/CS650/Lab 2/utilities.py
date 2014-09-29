'''
Created on Sep 28, 2014

@author: hcmi
'''
import csv
import numpy as np

def writeToCsv(filename, data):
    
    with open(filename, 'wb') as fp:
        a = csv.writer(fp, delimiter=',')
        a.writerows(data)
        
def readFromCsv(filename):
    
    data = []
    with open(filename, 'rb') as fp:
        a = csv.reader(fp)
        for row in a:
            row_data = []
            for r in row:
                row_data.append(float(r))
            data.append(row_data)
            
    return np.array(data)
